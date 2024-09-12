#include <esp_log.h>

#include "OLED.hpp"


namespace oled
{

bool OLED::send_cmd(uint8_t cmd)
{
    return i2c_write8(this->i2c_num,this->address,OLED_CMD_BYTE,cmd);
}

bool OLED::send_data(uint8_t data)
{
    return i2c_write8(this->i2c_num,this->address,OLED_DATA_BYTE,data);
}

void OLED::rewind_column()
{
    this->send_cmd(0x00); // lower address column address for page address set to 0
    this->send_cmd(0x10); // higher address column for page address set to 0
}

bool OLED::init(uint8_t width,uint8_t height)
{
    if((width!=128)||((height!=64)&&(height!=32)))
    {
        ESP_LOGE("OLED","Wrong display size!");
        return false;
    }

    bool res=true;

    switch(height)
    {
        case 32:

        for(const uint8_t& cmd : s_oled128x32_initData)
        {
            res=this->send_cmd(cmd);
        }

        break;

        case 64:

        for(const uint8_t& cmd : s_oled128x64_initData)
        {
            res=this->send_cmd(cmd);
        }

        break;
    }

    if(res)
    {
        this->width=width;
        this->height=height;

        buffer=new uint8_t[this->width*(this->height/8)]{0};
    }

    return res;

}

void OLED::drawPixel(const uint8_t& x,const uint8_t& y,const bool& color)
{
    if((x>=this->width)||(y>=this->height))
    {
        return;
    }

    uint8_t colPos= y/8;

    if(!color)
    {

        this->buffer[colPos*this->width + x]&=~(1 << y%8);
        
    }
    else
    {

        this->buffer[colPos*this->width + x]|=(1 << y%8);

    }

}

// needs some improvments
void OLED::drawLine(const uint8_t& x1,const uint8_t& y1,const uint8_t& x2,const uint8_t& y2,const bool& color)
{
    
    int8_t dx= x1 < x2 ? 1 : -1;
    int8_t dy= y1 < y2 ? 1 : -1;

    if(x1==x2)
    {
        for(uint8_t y=y1;y!=y2;y+=dy)
        {
            drawPixel(x1,y,color);
        }

        return;
    }

    if(y1==y2)
    {
        dy=0;
    }

    int m_new = 2 * abs(y2 - y1);
    int slope_error_new = m_new - abs(x2 - x1);
    for (int x = x1, y = y1; x != x2; x+=dx) {
        
        this->drawPixel(x,y,color);
 
        slope_error_new += m_new;

        if (slope_error_new >= 0) {
            y+=dy;
            slope_error_new -= 2 * abs(x2 - x1);
        }
    }
}

void OLED::drawRect(const uint8_t& left,const uint8_t& top,const uint8_t& right,const uint8_t& bottom,const bool& color)
{
    this->drawLine(left,top,right,top,color);
    this->drawLine(right,top,right,bottom,color);
    this->drawLine(right,bottom,left,bottom,color);
    this->drawLine(left,bottom,left,top,color);
}

void OLED::fillRect(const uint8_t& left,const uint8_t& top,const uint8_t& right,const uint8_t& bottom,const bool& color)
{
    for(uint8_t y=top;y<bottom;++y)
    {
        for(uint8_t x=left;x<right;++x)
        {
            this->drawPixel(x,y,color);
        }
    }
}

void OLED::fill(const uint8_t& color)
{
    this->fillRect(0,0,this->width,this->height,color);
}


void OLED::drawBitmap(const uint8_t& x,const uint8_t& y,const uint8_t& width,const uint8_t& height,const uint8_t* buffer)
{
    uint8_t _x=0;
    uint8_t _y=0;

    for(uint16_t u=0;u<height*width/8;++u)
    {
        for(uint8_t c=0;c<8;++c)
        {
            this->drawPixel(x+_x,y+_y,( buffer[u] & ( 1<< (7-c) )));
            ++_x;
        }

        if(_x==width)
        {
            ++_y;
            _x=0;
        }
    }
}

void OLED::drawChar(const uint8_t& x,const uint8_t& y,const bool& color,char text)
{
    if(this->font[0]!=0x00)
    {
        ESP_LOGE("OLED","Unsupported font type, require fixed type");
        return;
    }

    const uint8_t& font_height=this->font[2];
    const uint8_t& font_width=this->font[1];

    const char& first_char=this->font[3];

    // character is out of bounds
    if(text<first_char)
    {
        ESP_LOGE("OLED","Characet %c is out of bounds",text);
        return;
    }

    const uint32_t bytes_per_characer=font_height/8 + font_height%8 == 0 ? 0 : 1;

    const uint32_t char_index=( (text-first_char)*font_width*bytes_per_characer ) + 4;

    for(uint8_t d=0;d<font_width;++d)
    {

        for(uint8_t q=0;q<font_height;++q)
        {

            const uint8_t& c=this->font[char_index + d*bytes_per_characer + q/8];

            this->drawPixel(x+d,y+q,(( c & ( 1<< q%8 ) ))^ (!color));

        }

    }
}

void OLED::drawText(const uint8_t& x,const uint8_t& y,const bool& color,const char* text)
{
    if(this->font[0]!=0x00)
    {
        ESP_LOGE("OLED","Unsupported font type, require fixed type");
        return;
    }

    const uint8_t& font_height=this->font[2];
    const uint8_t& font_width=this->font[1]; 

    uint8_t _x=x;
    uint8_t _y=y;

    while(*text!=0)
    {
        this->drawChar(_x,_y,color,*text);

        _x+=font_width+1;

        if(_x>=(128-font_width))
        {
            _x=0;
            _y+=font_height+1;
        }

        ++text;
    }

}

// to do
bool OLED::draw()
{
    uint8_t page_num=this->height/8;
    for(uint8_t i=0;i<page_num;++i)
    {
        this->send_cmd(0xB0+i);
        this->rewind_column();

        for(uint8_t x=0;x<this->width;++x)
        {
        this->send_data(this->buffer[i*this->width+x]);
        }
    }

    return true;
}

};