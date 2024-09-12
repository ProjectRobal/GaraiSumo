#pragma once

#include <cstdint>

extern "C"
{

    #include "i2c.h"
    #include "oled/ssd1306_fonts.h"
    #include "oled/ssd1306_commands.h"
    
}

#define OLED_CMD_BYTE 0b00000000
#define OLED_DATA_BYTE 0b01000000

namespace oled
{

const uint8_t  s_oled128x64_initData[] =
{
    SSD1306_DISPLAYOFF, // display off
    SSD1306_MEMORYMODE, HORIZONTAL_ADDRESSING_MODE, // Page Addressing mode
    SSD1306_COMSCANDEC,             // Scan from 127 to 0 (Reverse scan)
    SSD1306_SETSTARTLINE | 0x00,    // First line to start scanning from
    SSD1306_SETCONTRAST, 0x7F,      // contast value to 0x7F according to datasheet
    SSD1306_SEGREMAP | 0x01,        // Use reverse mapping. 0x00 - is normal mapping
    SSD1306_NORMALDISPLAY,
    SSD1306_SETMULTIPLEX, 63,       // Reset to default MUX. See datasheet
    SSD1306_SETDISPLAYOFFSET, 0x00, // no offset
    SSD1306_SETDISPLAYCLOCKDIV, 0x80,// set to default ratio/osc frequency
    SSD1306_SETPRECHARGE, 0x22,     // switch precharge to 0x22 // 0xF1
    SSD1306_SETCOMPINS, 0x12,       // set divide ratio
    SSD1306_SETVCOMDETECT, 0x20,    // vcom deselect to 0x20 // 0x40
    SSD1306_CHARGEPUMP, 0x14,       // Enable charge pump
    SSD1306_DISPLAYALLON_RESUME,
    SSD1306_DISPLAYON,
};

const uint8_t  s_oled128x32_initData[] =
{
    SSD1306_DISPLAYOFF, // display off
    SSD1306_SETDISPLAYCLOCKDIV, 0x80,
    SSD1306_SETMULTIPLEX, 31,
    SSD1306_SETDISPLAYOFFSET, 0x00, // --no offset
    SSD1306_SETSTARTLINE | 0x00,
    SSD1306_CHARGEPUMP, 0x14, // 0x10
    SSD1306_SEGREMAP | 0x01,  // Reverse mapping
    SSD1306_COMSCANDEC,
    SSD1306_SETCOMPINS, 0x02,
    SSD1306_SETCONTRAST, 0x7F, // contast value
    SSD1306_SETPRECHARGE, 0x22, // 0x1F
    SSD1306_SETVCOMDETECT, 0x40,
    SSD1306_MEMORYMODE, HORIZONTAL_ADDRESSING_MODE,
    SSD1306_DISPLAYALLON_RESUME,
    SSD1306_NORMALDISPLAY,
    SSD1306_DISPLAYON,
};

class OLED
{
    private:

    uint8_t const  *font;

    uint8_t *buffer;

    const uint8_t address;

    const uint8_t i2c_num;

    uint8_t width;
    uint8_t height;

    bool send_cmd(uint8_t cmd);

    bool send_data(uint8_t data);

    void rewind_column();

    public:  

    OLED(i2c_port_t i2c_num,uint8_t address=0x3C)
    : address(address),
    i2c_num(i2c_num)
    {}

    bool init(uint8_t width,uint8_t height);

    void setFont(uint8_t const *font)
    {
        this->font=font;
    }

    void drawPixel(uint8_t x,uint8_t y,const bool& color);

    void drawLine(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2,const bool& color);

    void drawRect(uint8_t left,uint8_t top,uint8_t right,uint8_t bottom,const bool& color);

    void fillRect(uint8_t left,uint8_t top,uint8_t right,uint8_t bottom,const bool& color);

    void fill(uint8_t color);

    void clear()
    {
        this->fill(0);
    }

    void drawBitmap(uint8_t x,uint8_t y,uint8_t width,uint8_t height,const uint8_t* buffer);

    void drawText(uint8_t x,uint8_t y,const bool& color,const char* text);

    void drawChar(uint8_t x,uint8_t y,const bool& color,char text);

    // put buffer into RAM
    bool draw();

    uint8_t getWidth() const
    {
        return this->width;
    }

    uint8_t getHeight() const
    {
        return this->height;
    }
    

    ~OLED()
    {
        delete [] buffer;
    }

};

};