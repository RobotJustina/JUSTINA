/*
 * Texture.h
 *
 *  Created on: 10/04/2015
 *      Author: rey
 */
#pragma once

#include <string>
#include <GL/glew.h>
#include <FreeImage.h>

class Texture {
public:

	Texture(GLenum TextureTarget, const std::string& FileName);

	bool load();
	void bind(GLenum TextureUnit);
	virtual ~Texture();

	std::string getType() {
		return type;
	}

	void setType(std::string type) {
		this->type = type;
	}

	std::string getFileName() {
		return m_fileName;
	}

    int getWidth(){
        return width;
    }

    int getHeight(){
        return height;
    }

private:
	std::string m_fileName;
	GLenum m_textureTarget;
	GLuint m_textureObj;
	std::string type;
    int width;
    int height;
};
