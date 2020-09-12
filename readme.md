Xpressnet 
=====================

This code is modified code from http://pgahtow.de/wiki/index.php?title=Hauptseite for use on a STM32 Maple Mini.
The adapted XpressNet code is used in the [XMC Project](https://github.com/MDRRC/XMC)

If you want to use this code in your own project

 * Download [Stm32 Arduino library](https://github.com/rogerclarkmelbourne/Arduino_STM32), use the [1.0.0 release](https://github.com/rogerclarkmelbourne/Arduino_STM32/releases/tag/v1.0.0)! The most recent version at the master may give problems because of some changes.  
 * Remove or rename (.x for example) ..\Arduino_STM32-master\STM32F1\cores\maple\libmaple\usart.c so it's NOT build
 * Remove or rename (.x for example) ..\Arduino_STM32-master\STM32F1\cores\maple\libmaple\usart_f1.c so it's NOT build 
 * Remove or rename (.x for example) ..\Arduino_STM32-master\STM32F1\system\libmaple\usart_private.h so it's NOT build 

@ Copyright notice, files are changed by me to get XpressNet working on the STM32f1 series but all credits to the creators of XpressNet and Stm32 files for Arduino!
