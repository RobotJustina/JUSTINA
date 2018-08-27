#include "graphics_viz/Shader.h"

std::string Shader::loadShaderFile(std::string strFile) {
    std::ifstream fin(strFile.c_str());

    if (!fin)
        return "";

    std::string strLine = "";
    std::string strText = "";

    while (getline(fin, strLine)) {
        strText = strText + "\n" + strLine;
    }

    fin.close();

    return strText;
}

void Shader::initialize(std::string strVertexFile,
                        std::string strFragmentFile) {

    std::string strVShader, strFShader;

    if (!strVertexFile.length() || !strFragmentFile.length())
        return;

    if (VertexShaderId || FragmentShaderId || ShaderProgramId)
        destroy();

    GLenum ErrorCheckValue = glGetError();

    VertexShaderId = glCreateShader(GL_VERTEX_SHADER);
    FragmentShaderId = glCreateShader(GL_FRAGMENT_SHADER);

    strVShader = loadShaderFile(strVertexFile.c_str());
    strFShader = loadShaderFile(strFragmentFile.c_str());

    const char *szVShader = strVShader.c_str();
    const char *szFShader = strFShader.c_str();

    glShaderSource(VertexShaderId, 1, &szVShader, nullptr);
    glShaderSource(FragmentShaderId, 1, &szFShader, nullptr);

    GLint isCompiled = 0;
    glCompileShader(VertexShaderId);
    glGetShaderiv(VertexShaderId, GL_COMPILE_STATUS, &isCompiled);
    if (isCompiled == GL_FALSE) {
        GLint maxLength = 0;
        glGetShaderiv(VertexShaderId, GL_INFO_LOG_LENGTH, &maxLength);

        std::vector<GLchar> errorLog(maxLength);
        glGetShaderInfoLog(VertexShaderId, maxLength, &maxLength, &errorLog[0]);
        printf("%s\n", &errorLog[0]);

        glDeleteShader(VertexShaderId);
        return exit(-1);
    }
    glCompileShader(FragmentShaderId);
    glGetShaderiv(FragmentShaderId, GL_COMPILE_STATUS, &isCompiled);
    if (isCompiled == GL_FALSE) {
        GLint maxLength = 0;
        glGetShaderiv(FragmentShaderId, GL_INFO_LOG_LENGTH, &maxLength);

        std::vector<GLchar> errorLog(maxLength);
        glGetShaderInfoLog(FragmentShaderId, maxLength, &maxLength,
                           &errorLog[0]);
        printf("%s\n", &errorLog[0]);

        glDeleteShader(FragmentShaderId);
        return exit(-1);
    }

    ShaderProgramId = glCreateProgram();

    glAttachShader(ShaderProgramId, VertexShaderId);
    glAttachShader(ShaderProgramId, FragmentShaderId);

    glLinkProgram(ShaderProgramId);

    ErrorCheckValue = glGetError();

    if (ErrorCheckValue != GL_NO_ERROR) {
        fprintf(stderr,
                "ERROR: Could not create the shader program with error Id: %d\n",
                ErrorCheckValue);
        exit(-1);
    }
}

GLint Shader::getUniformLocation(std::string strVariable) {
    if (!ShaderProgramId)
        return -1;
    return glGetUniformLocation(ShaderProgramId, strVariable.c_str());
}

void Shader::destroy() {
    glUseProgram(0);

    if (VertexShaderId) {
        glDetachShader(ShaderProgramId, VertexShaderId);
        glDeleteShader(VertexShaderId);
        VertexShaderId = 0;
    }

    if (FragmentShaderId) {
        glDetachShader(ShaderProgramId, FragmentShaderId);
        glDeleteShader(FragmentShaderId);
        FragmentShaderId = 0;
    }

    if (ShaderProgramId) {
        glDeleteProgram(ShaderProgramId);
        ShaderProgramId = 0;
    }
}
