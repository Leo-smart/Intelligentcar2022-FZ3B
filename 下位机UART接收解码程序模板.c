

/**
* @brief        : 将整段代码拷贝至下位机程序中启用
* @author       : Leo
* @notes        : 接收并校验完成后，尽量不要在串口中断中进行后续操作
**/

#define USB_FRAME_HEAD				0x42			//USB通信序列帧头
#define USB_FRAME_END				0x10			//USB通信序列帧尾
#define USB_FRAME_LENGTH			12				//USB通信序列字节长度
typedef struct 
{
	bool ReceiveStart;								//数据接收开始
	uint8_t ReceiveIndex;						    //接收序列
	bool ReceiveFinished;							//数据队列接收并校验完成
	uint8_t ReceiveBuff[USB_FRAME_LENGTH];	        //USB接收队列：临时接收
	uint8_t ReceiveBuffFinished[USB_FRAME_LENGTH];	//USB接收队列：校验成功
}USB_STRUCT;

typedef union 
{
	unsigned char U8_Buff[4];
	float   	Float;
}UNION_BIT32;

USB_STRUCT usbStr;

/**
* @Description  : USB/UART通信协议接收校验
* @params       : data: 串口接收字节
* @Date         : 
* @author       : SASU/Leo
* @notes        : 将该API放置在串口接收中断中，
**/
void uart_receiveVerify(unsigned char data)
{ 
    if(data == USB_FRAME_HEAD && !usbStr.ReceiveStart)//监测帧头
    {
        usbStr.ReceiveStart = true;
        usbStr.ReceiveBuff[0] = data;
        usbStr.ReceiveIndex = 1;
    }
    else if(usbStr.ReceiveStart && usbStr.ReceiveIndex<USB_FRAME_LENGTH)//通信开始|接收队列数据
    {
        usbStr.ReceiveBuff[usbStr.ReceiveIndex] = data;
        usbStr.ReceiveIndex++;
    }
    
    if(usbStr.ReceiveIndex>=USB_FRAME_LENGTH)
    {
        if(usbStr.ReceiveBuff[USB_FRAME_LENGTH-1] == USB_FRAME_END)//帧尾
        {
            uint8_t check = 0;
            for(int i=0;i<10;i++)
                check += usbStr.ReceiveBuff[i];
            
            if(check == usbStr.ReceiveBuff[10])//校验位
            {
                memcpy(usbStr.ReceiveBuffFinished,usbStr.ReceiveBuff,USB_FRAME_LENGTH);	
                usbStr.ReceiveFinished = true;
                
                //通信成功！！！速度和方向控制特殊处理----------------------------------------------------------------------
                uint8_t Addr = (uint8_t)(usbStr.ReceiveBuffFinished[1] & 0xFF);
                uint8_t BuffData[8] = {0};
                UNION_BIT32 UnionBit32;
                for(int i=0;i<8;i++)
                {
                    BuffData[i] = usbStr.ReceiveBuffFinished[2+i];
                }
                switch(Addr)
                {
                        case 0x01:	//智能车速度,智能车姿态（方向）
                            UnionBit32.U8_Buff[0] = BuffData[0];
                            UnionBit32.U8_Buff[1] = BuffData[1];
                            UnionBit32.U8_Buff[2] = BuffData[2];
                            UnionBit32.U8_Buff[3] = BuffData[3];
                            float speed = UnionBit32.Float;	        //电机速度
                        
                            UnionBit32.U8_Buff[0] = BuffData[4];
                            UnionBit32.U8_Buff[1] = BuffData[5];
                            UnionBit32.U8_Buff[2] = BuffData[6];
                            UnionBit32.U8_Buff[3] = BuffData[7];
                            float angle = UnionBit32.Float;	        //舵机方向
                            break;
                }              
                //---------------------------------------------------------------------------------------------------          
            }
        }
        
        usbStr.ReceiveIndex = 0;
        usbStr.ReceiveStart = false;
    }
}