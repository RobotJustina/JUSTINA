/*
 * Texture.cpp
 *
 *  Created on: 10/04/2015
 *      Author: rey
 */
#include "graphics_viz/Texture.h"
#include <iostream>

Texture::Texture(GLenum TextureTarget, const std::string& FileName) :
		m_textureObj(0), type("") {
	this->m_textureTarget = TextureTarget;
	this->m_fileName = FileName;
}

Texture::~Texture() {
	//glDeleteTextures(1, &m_textureObj);
}

bool Texture::load() {
	const char* filename = m_fileName.c_str();

	// Determina el formato de la imagen
	FREE_IMAGE_FORMAT format = FreeImage_GetFileType(filename, 0);

	// Si la imagen no es encontrada termina el programa
	if (format == -1) {
		std::cout << "No se encontro la imagen: " << m_fileName
				<< " - Abortando." << std::endl;
		exit(-1);
	}
	// Si el formato no es soportado por FreeImage termina el programa
	if (format == FIF_UNKNOWN) {
		std::cout
				<< "No se puede determinar el formato de imagen - validarla extension del archivo..."
				<< std::endl;

		// Se obtiene el formato del archivo
		format = FreeImage_GetFIFFromFilename(filename);

		// Revisa si la libreria es capaz de leer el formato
		if (!FreeImage_FIFSupportsReading(format)) {
			std::cout << "El formato de la imagen no puede ser leeido!"
					<< std::endl;
			exit(-1);
		}
	}
	// Si es valida la imagen y puede ser leeido, se carga la imagen en un bitap
	FIBITMAP* bitmap = FreeImage_Load(format, filename);
	FreeImage_FlipVertical(bitmap);

	// Obtiene el numero de bits por pixel de la imagen
	int bitsPerPixel = FreeImage_GetBPP(bitmap);

	// Convierte la imagen a 32 bits (8 bits por canal).
	FIBITMAP* bitmap32;
	if (bitsPerPixel == 32) {
		/*std::cout << "Source image has " << bitsPerPixel
		 << " bits per pixel. Skipping conversion." << std::endl;*/
		bitmap32 = bitmap;
	} else {
		/*std::cout << "Source image has " << bitsPerPixel
		 << " bits per pixel. Converting to 32-bit colour." << std::endl;*/
		bitmap32 = FreeImage_ConvertTo32Bits(bitmap);
	}
	// Se obtiene las dimensiones de la imagen.
    width = FreeImage_GetWidth(bitmap32);
    height = FreeImage_GetHeight(bitmap32);
    /*std::cout << "Image: " << m_fileName << " is size: " << width << "x"
	 << imageHeight << "." << std::endl;*/

	// Se obtiene un apuntador a los datos de la textura como un arreglo de unsigned bytes.
	GLubyte* textureData = FreeImage_GetBits(bitmap32);

	// Se genera un buffer para textura en la GPU
	glGenTextures(1, &m_textureObj);
	glBindTexture(m_textureTarget, m_textureObj);

	// Se envian los datos de la textura
	glTexImage2D(GL_TEXTURE_2D, // Tipo de textura
			0, // Niveles del Mipmap
			GL_RGBA, //Formato intero, RGBA
            width, // Ancho de la textura
            height, // Ancho de la textura
			0, // Borde de la textura
			GL_BGRA, // Formato que se maneja la textura
			GL_UNSIGNED_BYTE, // Tipo de datos de la textura
			textureData); // Imagen que se usa para esta textura
	// Se indica el tipo de interpolacion para ajustar la imagen que se cargo a la GPU
    glTexParameterf(m_textureTarget, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameterf(m_textureTarget, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

	// Unload the 32-bit colour bitmap
	// Se desecha los datos de la textura
	FreeImage_Unload(bitmap32);

	// Si se convirtio la imagen a 32 bits por pixel eliminamos el bitmap
	if (bitsPerPixel != 32) {
		FreeImage_Unload(bitmap);
	}

	// Desenlazamos la textura
	glBindTexture(m_textureTarget, 0);
	return true;
}

void Texture::bind(GLenum TextureUnit) {
	// Se activan la unidade de textura para el objeto de textura con el que fue creado este objeto
	glActiveTexture(TextureUnit);
	glBindTexture(m_textureTarget, m_textureObj);
}
