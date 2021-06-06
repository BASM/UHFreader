# UHFreader

Library for UHF readers, tested:
* Chafon CF-RU5102
* LJYZN-105

Works fine base API:
* Inventory
* ReadCard
* WriteCard
* WriteEPC
* Lock


Works fine card API:
* GetReaderInformation
* Beep

Works fine hi level API:
* ReadCardTID
* ReadCardEPC
* ReadCardPass

Silabs Wiegand generator (by UART RTS, DTR) 34 bits: 
* WgSend


Not implemented:
* KillTag 
* BlockErase 
* All NXP UCODE tags:
** ReadProtect
** ProtectNoEPC
** ResetReadProtect
** CheckReadProtect
** EasAlarm
** CheckEasAlarm
** BlockLock
** InventorySingle
** BlcokWrite
* All 18000-6B COMMAND:
** InvSignal6B
** InvMulti6b
** ReadData6b
** WriteData6b
** CheckLock6b
** Lock6b
* Some reader funcitons:
** SetRegion
** SetAddress
** SetScantime
** BaudRate
** SetPower


