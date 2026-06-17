[CmdletBinding()]
param(
    [string]$RepositoryPath = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
)

$ErrorActionPreference = 'Stop'
$proposalDirectory = $PSScriptRoot
$workspaceFile = Join-Path $RepositoryPath 'catkin_ws\Justina.workspace'
$composeFile = Join-Path $proposalDirectory 'compose.yaml'
$dockerfile = Join-Path $proposalDirectory 'Dockerfile'

if (-not (Test-Path -LiteralPath $workspaceFile -PathType Leaf)) {
    throw "No se encontró el workspace: $workspaceFile"
}

[xml]$workspace = Get-Content -Raw -LiteralPath $workspaceFile
$distribution = $workspace.Workspace.Distribution.name
if ($distribution -ne 'kinetic') {
    throw "Se esperaba ROS Kinetic en Justina.workspace; se encontró: $distribution"
}

$dockerfileContent = Get-Content -Raw -LiteralPath $dockerfile
if ($dockerfileContent -notmatch 'LC_ALL=es_CO\.UTF-8') {
    throw 'El Dockerfile no configura LC_ALL=es_CO.UTF-8.'
}

$unsafePatterns = @('--privileged', 'apt-key', 'nvidia-docker', 'chown\s+-R\s+/home', 'rm\s+-rf\s+/opt/ros')
$implementationFiles = @(
    $composeFile,
    $dockerfile,
    (Join-Path $proposalDirectory 'container-entrypoint.sh'),
    (Join-Path $proposalDirectory 'build-workspace.sh')
)
foreach ($pattern in $unsafePatterns) {
    if (Select-String -Path $implementationFiles -Pattern $pattern) {
        throw "Se encontró un patrón heredado no permitido: $pattern"
    }
}

$previousRepoPath = $env:JUSTINA_REPO_PATH
try {
    $env:JUSTINA_REPO_PATH = $RepositoryPath
    if (Get-Command docker-compose -ErrorAction SilentlyContinue) {
        & docker-compose -f $composeFile config | Out-Null
    }
    else {
        & docker compose -f $composeFile config | Out-Null
    }
    if ($LASTEXITCODE -ne 0) {
        throw 'La validación de Docker Compose falló.'
    }
}
finally {
    $env:JUSTINA_REPO_PATH = $previousRepoPath
}

Write-Host 'Validación de la propuesta JUSTINA Colombia: correcta.'
