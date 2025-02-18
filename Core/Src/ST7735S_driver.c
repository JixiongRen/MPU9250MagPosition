//
// Created by renji on 25-2-18.
//
#include "ST7735S_driver.h"


void S_ST7735S_SetRST(uint8_t val) {
    if (val==0) HAL_GPIO_WritePin(LCD_RES_GPIO_Port, LCD_RES_Pin, GPIO_PIN_RESET);
    else HAL_GPIO_WritePin(LCD_RES_GPIO_Port, LCD_RES_Pin, GPIO_PIN_SET);
}

void S_ST7735S_SetDC(uint8_t val) {
    if (val==0) HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);
    else HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
}

void S_ST7735S_SendCommand(uint8_t cmd) {
    S_SPI_Start();
    S_ST7735S_SetDC(0);
    S_SPI_SwapByte(cmd);
    S_SPI_End();
}


void S_ST7735S_SendData(uint8_t data) {
    S_SPI_Start();
    S_ST7735S_SetDC(1);
    S_SPI_SwapByte(data);
    S_SPI_End();
}

void S_ST7735S_Send16bitsRGB(uint16_t rgb) {
    S_ST7735S_SendData(rgb >> 8);
    S_ST7735S_SendData(rgb);
}


void S_ST7735S_Init(void){
    S_Init_SPI();
    //除了上面SPI初始化的GPIO口,还需要额外初始化RST和DC
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = LCD_DC_Pin | LCD_RES_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(LCD_RES_GPIO_Port, &GPIO_InitStruct);
    
    S_ST7735S_SetRST(0);
    HAL_Delay(1);
    S_ST7735S_SetRST(1);
    HAL_Delay(120);
    //厂家提供的固定的初始化代码
    S_ST7735S_SendCommand(0x11); //Sleep out 
    HAL_Delay(120);              //Delay 120ms
    //------------------------------------ST7735S Frame Rate-----------------------------------------// 
    S_ST7735S_SendCommand(0xB1); 
    S_ST7735S_SendData(0x05); 
    S_ST7735S_SendData(0x3C); 
    S_ST7735S_SendData(0x3C); 
    S_ST7735S_SendCommand(0xB2); 
    S_ST7735S_SendData(0x05); 
    S_ST7735S_SendData(0x3C); 
    S_ST7735S_SendData(0x3C); 
    S_ST7735S_SendCommand(0xB3); 
    S_ST7735S_SendData(0x05); 
    S_ST7735S_SendData(0x3C); 
    S_ST7735S_SendData(0x3C); 
    S_ST7735S_SendData(0x05); 
    S_ST7735S_SendData(0x3C); 
    S_ST7735S_SendData(0x3C); 
    //------------------------------------End ST7735S Frame Rate---------------------------------// 
    S_ST7735S_SendCommand(0xB4); //Dot inversion 
    S_ST7735S_SendData(0x03); 
    //------------------------------------ST7735S Power Sequence---------------------------------// 
    S_ST7735S_SendCommand(0xC0); 
    S_ST7735S_SendData(0x28); 
    S_ST7735S_SendData(0x08); 
    S_ST7735S_SendData(0x04); 
    S_ST7735S_SendCommand(0xC1); 
    S_ST7735S_SendData(0XC0); 
    S_ST7735S_SendCommand(0xC2); 
    S_ST7735S_SendData(0x0D); 
    S_ST7735S_SendData(0x00); 
    S_ST7735S_SendCommand(0xC3); 
    S_ST7735S_SendData(0x8D); 
    S_ST7735S_SendData(0x2A); 
    S_ST7735S_SendCommand(0xC4); 
    S_ST7735S_SendData(0x8D); 
    S_ST7735S_SendData(0xEE); 
    //---------------------------------End ST7735S Power Sequence-------------------------------------// 
    S_ST7735S_SendCommand(0xC5); //VCOM 
    S_ST7735S_SendData(0x1A); 
    S_ST7735S_SendCommand(0x36); //MX, MY, RGB mode 
    S_ST7735S_SendData(0xC0); 
    //------------------------------------ST7735S Gamma Sequence---------------------------------// 
    S_ST7735S_SendCommand(0xE0); 
    S_ST7735S_SendData(0x04); 
    S_ST7735S_SendData(0x22); 
    S_ST7735S_SendData(0x07); 
    S_ST7735S_SendData(0x0A); 
    S_ST7735S_SendData(0x2E); 
    S_ST7735S_SendData(0x30); 
    S_ST7735S_SendData(0x25); 
    S_ST7735S_SendData(0x2A); 
    S_ST7735S_SendData(0x28); 
    S_ST7735S_SendData(0x26); 
    S_ST7735S_SendData(0x2E); 
    S_ST7735S_SendData(0x3A); 
    S_ST7735S_SendData(0x00); 
    S_ST7735S_SendData(0x01); 
    S_ST7735S_SendData(0x03); 
    S_ST7735S_SendData(0x13); 
    S_ST7735S_SendCommand(0xE1); 
    S_ST7735S_SendData(0x04); 
    S_ST7735S_SendData(0x16); 
    S_ST7735S_SendData(0x06); 
    S_ST7735S_SendData(0x0D); 
    S_ST7735S_SendData(0x2D); 
    S_ST7735S_SendData(0x26); 
    S_ST7735S_SendData(0x23); 
    S_ST7735S_SendData(0x27); 
    S_ST7735S_SendData(0x27); 
    S_ST7735S_SendData(0x25); 
    S_ST7735S_SendData(0x2D); 
    S_ST7735S_SendData(0x3B); 
    S_ST7735S_SendData(0x00); 
    S_ST7735S_SendData(0x01); 
    S_ST7735S_SendData(0x04); 
    S_ST7735S_SendData(0x13); 
    //------------------------------------End ST7735S Gamma Sequence-----------------------------// 
    S_ST7735S_SendCommand(0x3A); //65k mode 
    S_ST7735S_SendData(0x05); 
    S_ST7735S_SendCommand(0x29); //Display on 
}   


//指定范围
void S_ST7735S_SpecifyScope(uint8_t xs,uint8_t xe,uint8_t ys,uint8_t ye){
    S_ST7735S_SendCommand(0x2A);    //指定列范围
    S_ST7735S_SendData(0x00);
    S_ST7735S_SendData(xs);
    S_ST7735S_SendData(0x00);
    S_ST7735S_SendData(xe);
    
    S_ST7735S_SendCommand(0x2B);    //指定行范围
    S_ST7735S_SendData(0x00);
    S_ST7735S_SendData(ys);
    S_ST7735S_SendData(0x00);
    S_ST7735S_SendData(ye);
    
    S_ST7735S_SendCommand(0x2C);    //开始内存写入
}
 
 
void S_ST7735S_RefreshAll(uint16_t rgb){
    S_ST7735S_SpecifyScope(0,127,0,159);
    for(uint16_t j=0;j<160;++j){
        for(uint16_t i=0;i<128;++i){
            S_ST7735S_Send16bitsRGB(rgb);
        }
    }
}


// 显示单个ASCII字符
void S_ST7735S_DisplayChar(uint8_t x, uint8_t y, char ch, uint16_t color, uint16_t bgcolor, uint8_t fontSize) {
    const unsigned char *font;
    uint8_t width, height, bytesPerColumn;

    // 根据字体大小选择字库
    switch(fontSize) {
        case 12:
            font = asc2_1206[ch - 0x20];
            width = 6;
            height = 12;
            bytesPerColumn = 1;
            break;
        case 16:
            font = asc2_1608[ch - 0x20];
            width = 8;
            height = 16;
            bytesPerColumn = 2;
            break;
        case 24:
            font = asc2_2412[ch - 0x20];
            width = 12;
            height = 24;
            bytesPerColumn = 3;
            break;
        case 32:
            font = asc2_3216[ch - 0x20];
            width = 16;
            height = 32;
            bytesPerColumn = 4;
            break;
        default:
            return; // 不支持的字体大小
    }

    // 设置显示区域
    S_ST7735S_SpecifyScope(x, x + width - 1, y, y + height - 1);

    // 逐列绘制字符
    for(uint8_t col = 0; col < width; col++) {
        for(uint8_t row = 0; row < height; row++) {
            uint8_t byteIndex = col * bytesPerColumn + row / 8;
            uint8_t bitMask = 0x80 >> (row % 8);

            if(font[byteIndex] & bitMask) {
                S_ST7735S_Send16bitsRGB(color);
            } else {
                S_ST7735S_Send16bitsRGB(bgcolor);
            }
        }
    }
}

// 显示字符串
void S_ST7735S_DisplayString(uint8_t x, uint8_t y, const char *str, uint16_t color, uint16_t bgcolor, uint8_t fontSize) {
    uint8_t currentX = x;
    while(*str) {
        S_ST7735S_DisplayChar(currentX, y, *str, color, bgcolor, fontSize);

        // 根据字体宽度更新位置
        switch(fontSize) {
            case 12: currentX += 6; break;
            case 16: currentX += 8; break;
            case 24: currentX += 12; break;
            case 32: currentX += 16; break;
        }

        str++;
    }
}

// 显示单个数字（直接调用字符显示函数）
void S_ST7735S_DisplayNumber(uint8_t x, uint8_t y, uint8_t num, uint16_t color, uint16_t bgcolor, uint8_t fontSize) {
    if(num > 9) return;
    S_ST7735S_DisplayChar(x, y, num + '0', color, bgcolor, fontSize);
}