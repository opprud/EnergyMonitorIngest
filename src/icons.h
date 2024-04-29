#pragma once

#define ICON_SIZE 72

struct Bitmaps {
    static const uint8_t toaster[ICON_SIZE];
    static const uint8_t fridge[ICON_SIZE];
    static const uint8_t blender[ICON_SIZE];
    static const uint8_t coffee_maker[ICON_SIZE];
    static const uint8_t coffee[ICON_SIZE];
    static const uint8_t fan[ICON_SIZE];
    static const uint8_t lamp[ICON_SIZE];
    static const uint8_t anomaly[288];
    static const uint8_t plug[288];
};

const uint8_t Bitmaps::toaster[ICON_SIZE] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0x80, 0x07, 0xff, 0xe0, 0x06, 
    0x00, 0x60, 0x07, 0x30, 0xe0, 0x03, 0x34, 0xc0, 0x03, 0x00, 0xc0, 0x0f, 0xff, 0xf0, 0x1f, 0xff, 
    0xf8, 0x3f, 0xff, 0xfc, 0x3f, 0xff, 0xfc, 0x3f, 0xff, 0xfc, 0x3f, 0xff, 0xff, 0x3f, 0xff, 0xff, 
    0x3f, 0xff, 0xfc, 0x3f, 0xff, 0xcc, 0x3f, 0xff, 0xcc, 0x3f, 0xff, 0xfc, 0x3f, 0xff, 0xfc, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const uint8_t Bitmaps::fridge[ICON_SIZE] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xe0, 0x0f, 0xff, 0xf0, 0x0c, 0x00, 0x30, 0x0c, 
    0x00, 0x30, 0x0c, 0xc0, 0x30, 0x0c, 0xc0, 0x30, 0x0c, 0x00, 0x30, 0x0c, 0x00, 0x30, 0x0f, 0xff, 
    0xf0, 0x0f, 0xff, 0xf0, 0x0c, 0x00, 0x30, 0x0c, 0x00, 0x30, 0x0c, 0xc0, 0x30, 0x0c, 0xc0, 0x30, 
    0x0c, 0xc0, 0x30, 0x0c, 0xc0, 0x30, 0x0c, 0x00, 0x30, 0x0c, 0x00, 0x30, 0x0f, 0xff, 0xf0, 0x07, 
    0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};


const uint8_t Bitmaps::blender[ICON_SIZE] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff,0xf8, 0x1f, 0xff, 0xfc, 0x1f, 0xff, 0xcc, 0x0f, 
    0xff, 0xcc, 0x0f, 0xf9, 0xcc, 0x07, 0xf9, 0xcc, 0x07, 0xf9, 0xcc, 0x07, 0xf9, 0xcc, 0x07, 0xf9, 
    0xcc, 0x07, 0xf9, 0xc0, 0x07, 0xff, 0xc0, 0x03, 0xff, 0xc0, 0x03, 0xff, 0xc0, 0x03, 0xff, 0xc0, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xe0, 0x0f, 0xe7, 0xf0, 0x0f, 0xe7, 0xf0, 0x0f, 
    0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};


const uint8_t Bitmaps::coffee_maker[ICON_SIZE] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xc0, 0x1f, 0xff, 0xf0, 0x1f, 
    0xff, 0xf0, 0x1f, 0xff, 0xf8, 0x1f, 0xff, 0xf8, 0x1f, 0xff, 0xf8, 0x01, 0x87, 0xf8, 0x01, 0x87, 
    0xf8, 0x00, 0x07, 0xf8, 0x00, 0x07, 0xf8, 0x00, 0x07, 0xf8, 0x07, 0xe7, 0xf8, 0x07, 0xe7, 0x38, 
    0x07, 0xe7, 0x38, 0x07, 0xe7, 0xf8, 0x03, 0xc7, 0xf8, 0x1f, 0xff, 0xf8, 0x1f, 0xff, 0xf0, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};


const uint8_t Bitmaps::coffee[ICON_SIZE] = {
    0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0xcc, 0x00, 0x00, 0xcc, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x0f, 0xff, 0xc0, 0x0f, 0xff, 0xc0, 0x0f, 0xff, 0xf0, 0x07, 0xff, 0xf8, 0x07, 0xff, 
    0x98, 0x07, 0xff, 0x98, 0x07, 0xff, 0xf0, 0x07, 0xff, 0xe0, 0x03, 0xff, 0x00, 0x03, 0xff, 0x00, 
    0x03, 0xff, 0x00, 0x01, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xe0, 0x1f, 
    0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const uint8_t Bitmaps::fan[ICON_SIZE] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7e, 0x00, 0x01, 0xff, 0x80, 0x03, 0x81, 0xc0, 0x06, 
	0x00, 0x60, 0x0c, 0x70, 0x30, 0x18, 0xf0, 0x18, 0x18, 0xf8, 0x18, 0x30, 0x78, 0x0c, 0x30, 0x20, 
	0x0c, 0x30, 0x1b, 0xcc, 0x30, 0x5b, 0xcc, 0x30, 0xe7, 0xcc, 0x31, 0xe3, 0x8c, 0x19, 0xe0, 0x18, 
	0x18, 0xe0, 0x18, 0x0c, 0x00, 0x30, 0x06, 0x00, 0x60, 0x03, 0x81, 0xc0, 0x01, 0xff, 0x80, 0x00, 
	0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const uint8_t Bitmaps::lamp[ICON_SIZE] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x80, 0x00, 0x03, 0x83, 0xc0, 0x03, 0xff, 0xc0, 0x07, 
	0xff, 0xc0, 0x0f, 0xe3, 0xc0, 0x1f, 0xf0, 0xc0, 0x1f, 0xf0, 0xe0, 0x1f, 0xf0, 0xe0, 0x1f, 0xf0, 
	0x60, 0x00, 0x00, 0x78, 0x00, 0x00, 0x78, 0x00, 0x00, 0x78, 0x00, 0x00, 0xf8, 0x00, 0x00, 0xe0, 
	0x00, 0x01, 0xc0, 0x00, 0x03, 0x80, 0x00, 0x07, 0x00, 0x00, 0x07, 0x00, 0x00, 0x7f, 0xc0, 0x00, 
	0x7f, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const uint8_t Bitmaps::anomaly[288] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x07, 0xe0, 0x00, 0x00, 
	0x00, 0x00, 0x0f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xf8, 
	0x00, 0x00, 0x00, 0x00, 0x3f, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xfc, 0x00, 0x00, 0x00, 0x00, 
	0x7f, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xfe, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00, 
	0x00, 0x01, 0xff, 0xff, 0x80, 0x00, 0x00, 0x01, 0xff, 0xff, 0x80, 0x00, 0x00, 0x03, 0xfc, 0x3f, 
	0xc0, 0x00, 0x00, 0x03, 0xfc, 0x3f, 0xc0, 0x00, 0x00, 0x07, 0xfc, 0x3f, 0xe0, 0x00, 0x00, 0x0f, 
	0xfc, 0x3f, 0xf0, 0x00, 0x00, 0x0f, 0xfc, 0x3f, 0xf0, 0x00, 0x00, 0x1f, 0xfc, 0x3f, 0xf8, 0x00, 
	0x00, 0x1f, 0xfc, 0x3f, 0xf8, 0x00, 0x00, 0x3f, 0xfc, 0x3f, 0xfc, 0x00, 0x00, 0x7f, 0xfc, 0x3f, 
	0xfe, 0x00, 0x00, 0x7f, 0xfc, 0x3f, 0xfe, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0xff, 
	0xff, 0xff, 0xff, 0x00, 0x01, 0xff, 0xff, 0xff, 0xff, 0x80, 0x03, 0xff, 0xff, 0xff, 0xff, 0xc0, 
	0x03, 0xff, 0xfc, 0x3f, 0xff, 0xc0, 0x07, 0xff, 0xfc, 0x3f, 0xff, 0xe0, 0x07, 0xff, 0xfc, 0x3f, 
	0xff, 0xe0, 0x0f, 0xff, 0xfc, 0x3f, 0xff, 0xf0, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x1f, 0xff, 
	0xff, 0xff, 0xff, 0xf8, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xf8, 
	0x1f, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const uint8_t Bitmaps::plug[288] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x03, 0xc0, 0x00, 
	0x00, 0x00, 0x00, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x1f, 0xff, 
	0xc0, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xc0, 0x00, 0x00, 0x00, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x01, 
	0xff, 0xff, 0xc0, 0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x03, 0xff, 0xff, 0xff, 0xc0, 
	0x00, 0x03, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x03, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x03, 0xff, 0xff, 
	0xc0, 0x00, 0x00, 0x03, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xc0, 0x00, 0x00, 0xff, 
	0xff, 0xff, 0xc0, 0x00, 0x01, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x01, 0xff, 0xff, 0xff, 0xc0, 0x00, 
	0x03, 0xe3, 0xff, 0xff, 0xc0, 0x00, 0x03, 0xc3, 0xff, 0xff, 0xc0, 0x00, 0x03, 0xc3, 0xff, 0xff, 
	0xff, 0xc0, 0x03, 0xc3, 0xff, 0xff, 0xff, 0xc0, 0x03, 0xc3, 0xff, 0xff, 0xff, 0xc0, 0x03, 0xc1, 
	0xff, 0xff, 0xff, 0xc0, 0x03, 0xc1, 0xff, 0xff, 0xc0, 0x00, 0x03, 0xc0, 0xff, 0xff, 0xc0, 0x00, 
	0x03, 0xc0, 0x7f, 0xff, 0xc0, 0x00, 0x03, 0xc0, 0x1f, 0xff, 0xc0, 0x00, 0x03, 0xc0, 0x00, 0x03, 
	0xc0, 0x00, 0x03, 0xc0, 0x00, 0x03, 0xc0, 0x00, 0x03, 0xc0, 0x00, 0x03, 0xc0, 0x00, 0x03, 0xc0, 
	0x00, 0x03, 0xc0, 0x00, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 
	0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};