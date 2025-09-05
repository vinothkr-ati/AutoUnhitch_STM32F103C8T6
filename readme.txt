




Auto_Unhitch_RX_CANID 0x11155



byte [0] = AutoUnhitch PWM Duty Cycle Percentage

byte [1] = Auto Unhith Direction [ 0 / 1]





///////////////////////////////////////////////////////////////////////
Auto_Unhitch_TX_CANID 0x33333

byte [0] = Data /10  = ISEN Current

byte [1] = Data   = ADC 

byte [2] = Data   = ADC

byte [3] = Data /10  = VSEN_24V

byte [4] = 8 bits =  { 7, 6, 5 ,4 ,3 ,2 ,1, 0 }

bit 0 = Digital sense 1

bit 1 = Digital sense 2

bit 2 = Autounhitch Direction


byte [5] = Data  = PWM Feedback  (0 to 100%)

byte [6] = 8 bits  = { 7, 6, 5 ,4 ,3 ,2 ,1, 0 }

bit 0 = ISEN ERROR

bit 1 = nFault

