# Propuesta de contenedor JUSTINA para Colombia

Esta propuesta conserva Ubuntu 16.04 y ROS Kinetic dentro de un contenedor. No
ejecuta los instaladores heredados ni modifica paquetes del equipo anfitrión.

## Alcance

- Usa la imagen legada `ros:kinetic-ros-base-xenial` e instala el entorno
  completo `ros-kinetic-desktop-full`.
- Declara explícitamente la plataforma `linux/amd64`.
- Configura `America/Bogota` y `es_CO.UTF-8`.
- Monta el clon de JUSTINA en `/workspace/JUSTINA`.
- Ejecuta como usuario sin privilegios.
- No incluye CUDA, cámaras, USB ni acceso privilegiado por defecto.

La imagen depende de componentes sin soporte. Debe usarse para recuperación,
evaluación y migración, no como plataforma nueva de producción.
Si la imagen pública deja de estar disponible, configure `ROS_BASE_IMAGE` para
usar una copia aprobada en un registro interno.

## Uso

1. Entre a `dockers/justina-colombia`.
2. Cree la configuración local:

   ```bash
   cd dockers/justina-colombia
   cp .env.example .env
   ```

3. Cambie `JUSTINA_REPO_PATH` en `.env` por la ruta absoluta del repositorio.
4. Construya y abra el entorno:

   ```bash
   docker compose build
   docker compose run --rm justina
   ```

5. Dentro del contenedor, prepare dependencias y compile el workspace:

   ```bash
   build-justina-workspace
   ```

El comando incluye distribuciones ROS fuera de soporte al actualizar `rosdep`,
instala las dependencias declaradas, limpia resultados anteriores y compila.
Omite `libopenni-sensor-primesense-dev` porque ya no está disponible en los
repositorios de Ubuntu Xenial. Los módulos compilan con `libopenni`, pero usar
un sensor PrimeSense antiguo requiere instalar su controlador manualmente.

En Windows, después de aplicar la propuesta, valide los archivos con:

```powershell
.\dockers\justina-colombia\validate-proposal.ps1 -RepositoryPath .
```

## Issue #13

El archivo `catkin_ws/Justina.workspace` declara ROS Indigo, mientras los scripts
del repositorio instalan ROS Kinetic. Esta propuesta actualiza esa metadata a
Kinetic.

## Hardware y GPU

No habilite `--privileged` como solución general. Agregue dispositivos concretos
solo después de identificarlos, por ejemplo `/dev/video0` o un puerto serial.

Para GPU, instale NVIDIA Container Toolkit en el anfitrión y cree una variante
separada de Compose. CUDA 8 y las GPU modernas pueden ser incompatibles.

## Validación antes de revisión

```bash
docker compose config
docker build --check .
shellcheck container-entrypoint.sh
grep -Fq '<Distribution name="kinetic"/>' ../../catkin_ws/Justina.workspace
```

Estas validaciones requieren Docker Compose v2 y un anfitrión Linux. Docker
Desktop para Windows puede inspeccionar el archivo, pero `network_mode: host` y
el montaje de X11 están diseñados para Linux.
