#pragma once

#include <stdio.h>
#include "ImageData.h"

const int BYTES_PER_PIXEL = 3; /// red, green, & blue
const int FILE_HEADER_SIZE = 14;
const int INFO_HEADER_SIZE = 40;

void generateBitmapImage(ImageData& id, char *imageFileName);
unsigned char *createBitmapFileHeader(int height, int stride);
unsigned char *createBitmapInfoHeader(int height, int width);

unsigned int HDRToSDR(float hdr)
{
  float tone_mapped = hdr;
  float capped = tone_mapped > 1 ? 1 : tone_mapped;
  return 255 * capped;
}

void generateBitmapImage(ImageData &id, const char *imageFileName)
{

  unsigned char *image_flat = new unsigned char[id.h * id.w * BYTES_PER_PIXEL];

  for (int i = 0; i < id.data.size(); i++) {
    image_flat[i * 3 + 2] = HDRToSDR(id.data[i][0]); ///red
    image_flat[i * 3 + 1] = HDRToSDR(id.data[i][1]); ///green
    image_flat[i * 3 + 0] = HDRToSDR(id.data[i][2]); ///blue
  }

  int widthInBytes = id.w * BYTES_PER_PIXEL;

  unsigned char padding[3] = { 0, 0, 0 };
  int paddingSize = (4 - (widthInBytes) % 4) % 4;

  int stride = (widthInBytes)+paddingSize;

  FILE *imageFile = fopen(imageFileName, "wb");

  unsigned char *fileHeader = createBitmapFileHeader(id.h, stride);
  fwrite(fileHeader, 1, FILE_HEADER_SIZE, imageFile);

  unsigned char *infoHeader = createBitmapInfoHeader(id.h, id.w);
  fwrite(infoHeader, 1, INFO_HEADER_SIZE, imageFile);

  for (int i = 0; i < id.h; i++) {
    fwrite(image_flat + (i * widthInBytes), BYTES_PER_PIXEL, id.w, imageFile);
    fwrite(padding, 1, paddingSize, imageFile);
  }

  fclose(imageFile);

  delete[] image_flat;
}

unsigned char *createBitmapFileHeader(int height, int stride)
{
  int fileSize = FILE_HEADER_SIZE + INFO_HEADER_SIZE + (stride * height);

  static unsigned char fileHeader[] = {
      0,0,     /// signature
      0,0,0,0, /// image file size in bytes
      0,0,0,0, /// reserved
      0,0,0,0, /// start of pixel array
  };

  fileHeader[0] = (unsigned char)('B');
  fileHeader[1] = (unsigned char)('M');
  fileHeader[2] = (unsigned char)(fileSize);
  fileHeader[3] = (unsigned char)(fileSize >> 8);
  fileHeader[4] = (unsigned char)(fileSize >> 16);
  fileHeader[5] = (unsigned char)(fileSize >> 24);
  fileHeader[10] = (unsigned char)(FILE_HEADER_SIZE + INFO_HEADER_SIZE);

  return fileHeader;
}

unsigned char *createBitmapInfoHeader(int height, int width)
{
  static unsigned char infoHeader[] = {
      0,0,0,0, /// header size
      0,0,0,0, /// image width
      0,0,0,0, /// image height
      0,0,     /// number of color planes
      0,0,     /// bits per pixel
      0,0,0,0, /// compression
      0,0,0,0, /// image size
      0,0,0,0, /// horizontal resolution
      0,0,0,0, /// vertical resolution
      0,0,0,0, /// colors in color table
      0,0,0,0, /// important color count
  };

  infoHeader[0] = (unsigned char)(INFO_HEADER_SIZE);
  infoHeader[4] = (unsigned char)(width);
  infoHeader[5] = (unsigned char)(width >> 8);
  infoHeader[6] = (unsigned char)(width >> 16);
  infoHeader[7] = (unsigned char)(width >> 24);
  infoHeader[8] = (unsigned char)(height);
  infoHeader[9] = (unsigned char)(height >> 8);
  infoHeader[10] = (unsigned char)(height >> 16);
  infoHeader[11] = (unsigned char)(height >> 24);
  infoHeader[12] = (unsigned char)(1);
  infoHeader[14] = (unsigned char)(BYTES_PER_PIXEL * 8);

  return infoHeader;
}