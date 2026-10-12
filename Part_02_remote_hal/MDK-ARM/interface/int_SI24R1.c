#include "int_SI24R1.h"

//定义了一个静态发送地址 =》 发送地址与接收地址相同
uint8_t  TX_ADDRESS[TX_ADR_WIDTH] = {0x0A,0x01,0x06,0x0E,0x01};  // 定义了一个静态发送地址


//SPI读写一个字节函数 写入的字节是传入的参数 返回读取的字节
static uint8_t SPI_RW(uint8_t byte)
{
	uint8_t received_byte = 0;
	
	// SPI传输逻辑实现
	// 这里需要根据具体的硬件平台实现SPI读写操作
	HAL_SPI_TransmitReceive(&hspi1, &byte, &received_byte, 1, 1000);
	
	return received_byte;
}




/********************************************************
函数功能：写寄存器一个字节                
入口参数：reg:寄存器映射地址（格式：SI24R1_WRITE_REG|reg）
					value:寄存器的值
返回值：状态寄存器的值
*********************************************************/
uint8_t Int_SI24R1_WRITE_REG(uint8_t reg, uint8_t value)
{
	uint8_t status;

	CSN_LOW;                 
	status = SPI_RW(reg);				
	SPI_RW(value);
	CSN_HIGH; 
	
	return(status);
}


/********************************************************
函数功能：写寄存器多个字节                  
入口参数：reg:寄存器映射地址（格式：SI24R1_WRITE_REG|reg）
					pBuf:写入的数据首地址
					size:写入的数据字节数
返回值：状态寄存器的值
*********************************************************/
uint8_t Int_SI24R1_Write_Buf(uint8_t reg, const uint8_t *pBuf, uint8_t size)
{
	uint8_t status,byte_ctr;

  	CSN_LOW;                                  			
  	status = SPI_RW(reg);                          
  	for(byte_ctr=0; byte_ctr<size; byte_ctr++)
	{
		SPI_RW(*pBuf++);
	}     
  	CSN_HIGH;                                    	

  	return(status);       
}							  					   


/********************************************************
函数功能：读取寄存器一个字节                  
入口参数：reg:寄存器映射地址（格式：SI24R1_READ_REG|reg）
返回值：寄存器的值
*********************************************************/
uint8_t Int_SI24R1_READ_REG(uint8_t reg)
{
 	uint8_t value;

	CSN_LOW;   
	SPI_RW(reg);			
	value = SPI_RW(0);
	CSN_HIGH;              

	return(value);
}


/********************************************************
函数功能：读取寄存器多个字节                  
入口参数：reg:寄存器映射地址（格式：SI24R1_READ_REG|reg）
					pBuf:接收数据缓冲区的首地址
					size:读取字节数
返回值：状态寄存器的值
*********************************************************/
uint8_t Int_SI24R1_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t size)
{
	uint8_t status,byte_ctr;

  CSN_LOW;                                       
  status = SPI_RW(reg);                           
  for(byte_ctr=0;byte_ctr<size;byte_ctr++)
  {
	pBuf[byte_ctr] = SPI_RW(0);
  }
  CSN_HIGH;                                        

  return(status);    
}


/********************************************************
函数功能：SI24R1接收模式初始化                      
入口参数：无
返回值：无
*********************************************************/
void Int_SI24R1_RX_Mode(void)
{
	CE_LOW;
	Int_SI24R1_Write_Buf(SI24R1_WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);	// 接收设备接收通道0使用和发送设备相同的发送地址
	Int_SI24R1_WRITE_REG(SI24R1_WRITE_REG + EN_AA, 0x01);               			// 使能接收通道0自动应答
	Int_SI24R1_WRITE_REG(SI24R1_WRITE_REG + EN_RXADDR, 0x01);           			// 使能接收通道0
	Int_SI24R1_WRITE_REG(SI24R1_WRITE_REG + RF_CH, SI24R1_CHANNEL);                 // 选择射频通道40
	Int_SI24R1_WRITE_REG(SI24R1_WRITE_REG + RX_PW_P0, TX_PLOAD_WIDTH);  			// 接收通道0选择和发送通道相同的有效数据宽度
	Int_SI24R1_WRITE_REG(SI24R1_WRITE_REG + RF_SETUP, 0x06);            			// 数据传输率1Mbps，发射功率4dBm
	Int_SI24R1_WRITE_REG(SI24R1_WRITE_REG + CONFIG, 0x0f);              			// CRC使能，16位CRC校验，上电，接收模式
	Int_SI24R1_WRITE_REG(SI24R1_WRITE_REG + STATUS, 0xff);  						//清除所有的中断标志位
	CE_HIGH;                                           								// 使能CE，接收设备
}						


/********************************************************
函数功能：SI24R1发送模式初始化                      
入口参数：无
返回值：无
*********************************************************/
void Int_SI24R1_TX_Mode(void)
{
	CE_LOW;
	Int_SI24R1_Write_Buf(SI24R1_WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);     // 写入发送地址
	Int_SI24R1_Write_Buf(SI24R1_WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);  // 为了应答接收设备，接收通道0地址和发送地址相同

	Int_SI24R1_WRITE_REG(SI24R1_WRITE_REG + EN_AA, 0x01);       					// 使能接收通道0自动应答
	Int_SI24R1_WRITE_REG(SI24R1_WRITE_REG + EN_RXADDR, 0x01);   					// 使能接收通道0
	Int_SI24R1_WRITE_REG(SI24R1_WRITE_REG + SETUP_RETR, 0x0a);  					// 自动重发等待250us+86us，自动重发10次
	Int_SI24R1_WRITE_REG(SI24R1_WRITE_REG + RF_CH, SI24R1_CHANNEL);         		// 选择射频通道0x40
	Int_SI24R1_WRITE_REG(SI24R1_WRITE_REG + RF_SETUP, 0x06);    					// 数据传输率1Mbps，发射功率4dBm
	Int_SI24R1_WRITE_REG(SI24R1_WRITE_REG + CONFIG, 0x0e);      					// CRC使能，16位CRC校验，上电
	CE_HIGH;
}


/********************************************************
函数功能：读取接收数据    由硬件直接接受保存到 FIFO队列中 通过状态标志位判断队列中是否有数据                   
入口参数：rxbuf:接收数据存放首地址
返回值：0:接收到数据
          1:没有接收到数据
*********************************************************/
uint8_t Int_SI24R1_RxPacket(uint8_t *rxbuf)
{
	uint8_t state;
	//将读取到的值，原封不动的写入到状态寄存器中 =》因为状态寄存器中的标志位设计为写1清除
	state = Int_SI24R1_READ_REG(STATUS);  			                 					//读取状态寄存器的值    	  
	Int_SI24R1_WRITE_REG(SI24R1_WRITE_REG + STATUS, state);               			//清除RX_DS中断标志

	if(state & RX_DR)								                       			//接收到数据
	{
		Int_SI24R1_Read_Buf(RD_RX_PLOAD,rxbuf,TX_PLOAD_WIDTH);     					//读取数据,0X61并非地址，而是SPI命令
		Int_SI24R1_WRITE_REG(FLUSH_RX,0xff);					            		//清除RX FIFO寄存器
		return 0; 
	}	   
	return 1;                                                   					//没有收到任何数据
}


/********************************************************
函数功能：发送一个数据包                      
入口参数：txbuf:要发送的数据
返回值：0:发送成功
		  1:发送失败               
*********************************************************/
uint8_t Int_SI24R1_TxPacket(uint8_t *txbuf)
{
	uint8_t state;
	CE_LOW;																			//CE拉低，使SI24R1停止发送
  	Int_SI24R1_Write_Buf(WR_TX_PLOAD, txbuf, TX_PLOAD_WIDTH);	    				//写数据到TX FIFO,32个字节
 	CE_HIGH;																		//CE拉高，使能发送	   
	
	//不使用中断标志位判断发送是否完成，使用轮询读取状态标志位
	//while(IRQ == 1);																//等待发送完成


	state = Int_SI24R1_READ_REG(STATUS);  											//读取状态寄存器的值	   

	while ((state & TX_DS) == 0 && (state & MAX_RT) == 0)
	{
		state = Int_SI24R1_READ_REG(STATUS);
		vTaskDelay(1);
	}
	


	Int_SI24R1_WRITE_REG(SI24R1_WRITE_REG+STATUS, state); 							//清除TX_DS或MAX_RT中断标志
	if(state&MAX_RT)																//达到最大重发次数
	{
		Int_SI24R1_WRITE_REG(FLUSH_TX,0xff);										//清除TX FIFO寄存器 
		return 1; 
	}
	if(state&TX_DS)																	//发送完成
	{
		return 0;
	}
	return 1;																		//发送失败
}



uint8_t si24r1_rx_buf[5] = {0};
//检测SI24R1是否正常工作
//返回值：0，正常
//		  1，异常
uint8_t Int_SI24R1_Check(void)
{
	
	//1.测试SPI1能否读写寄存器
	//1.0si24r1要先读取一次，保证SPI正常通讯才能写入
	Int_SI24R1_Read_Buf(SI24R1_READ_REG + RX_ADDR_P0, si24r1_rx_buf, TX_ADR_WIDTH);

	//1.1写入发送地址
	Int_SI24R1_Write_Buf(SI24R1_WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);

	//1.2读取同样的数据
	Int_SI24R1_Read_Buf(SI24R1_READ_REG + RX_ADDR_P0, si24r1_rx_buf, TX_ADR_WIDTH);
	
	for (uint8_t i = 0; i < TX_ADR_WIDTH; i++)
	{
		if (si24r1_rx_buf[i] != TX_ADDRESS[i])
		{
			return 1;
		}
		
	}
	return 0;
	
}


//硬件接口层SI24R1初始化
void Int_SI24R1_Init(void)
{
	//上电延时>100ms
	HAL_Delay(200);
	//1.校验检测
	if (Int_SI24R1_Check() == 1)
	{
		HAL_Delay(10);
	}

	//设置默认状态为接收模式 =》每次发送的时候切换到发送模式，发送完成后切换回接收模式
	Int_SI24R1_RX_Mode();
	debug_printf("SI24R1 init success\r\n");
}


