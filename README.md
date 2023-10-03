# Wireless_Charger
- zde naleznete instrukce, jak ovládat nabíječku přes UART.

## Zráva od nabíječky:

- Nabíječka posílá jednu zprávu každou 1 s pořád dokola. 
- Velikost zprávy: 7 bytů.

1. byte = 0xFF -> vždy, pro synchronizaci
2. byte error status:
	- 0x01 - load disconnected
	- 0x02 - transmitter dislocated / error in internal comm.
	- 0x03 - receiver ouput overvoltage	
	- 0x04 - receiver overcurrent	
	- 0x05 - transmitter overcurrent
	- 0x06 - transmitter input overvoltage
3. byte -> current state:
 	INITIAL_STATE = 1, 
	ANALOG_PING = 2, 
	DIGITAL_PING = 3, 
	COMM_ONLY_STATE = 4,
	POWER_TRANSFER_STATE = 6, 
	FAULT_STATE = 7 

- 4.-5. byte -> unsigned int - output current (v mA)
- 6.-7. byte -> unsigned int - output voltage (v mV)

data jsou seřazena viz následující kód:
	
	UART_BUFFER[3] = (uint8_t)(output_current & 0xFF); 	// Low byte of output_current								
	UART_BUFFER[4] = (uint8_t)((output_current >> 8) & 0xFF);    // High byte of output_current
	UART_BUFFER[5] = (uint8_t)(output_voltage & 0xFF);           // Low byte of output_voltage
	UART_BUFFER[6] = (uint8_t)((output_voltage >> 8) & 0xFF);    // High byte of output_voltage

## Příchozí zprávy:

- velikost 1 byte
- čte jako char, viz kód 
- celkem 5 možných zpráv podle charu:

- "P" ---> pokud je v COMM_ONLY_STATE a error_byte == 0 (žádná chyba), zahájí nabíjení = přechod do POWER_TRANSFER_STATE
- "p" ---> ukončí nabíjení, přechod z POWER_TRANSFER_STATE do COMM_ONLY_STATE
- "e" ---> vyřeší chyby = vynuluje error log ---> teprve potom je možné zahájit nabíjení
- "o" ---> ukončí vysílání kompletně a přejde do Ping Phase. Používat v případě, že chcete vysílač oddálit od přijímače (start rakety, servis atd.)
- "a" ---> znovu povolí komunikaci (přejde z Ping Phase do COMM_ONLY_STATE) - použít pouze v případě, že bylo předtím vysláno "o".... edge case


## Jak používat nabíječku:
1. nejprve přečíst zprávu, jestli je 2.byte = 0x00 -> žádný error
2. Nastavit výkon DC/DC měniče na cca 3 W 
3. Zkontrolovat, že se nabíječka nachází ve stavu COMM_ONLY_STATE
4. Poslat "P" na zahájení přenosu výkonu
5. Pro ukončení nabíjení poslat "p" - v případě, že je odebíraný proud přilíš malý (<1 mA), nabíječka se sama odpojí a přechází do COMM_ONLY_STATE.
6. Kontrolovat error_log, jestli nedošlo k chybě jako např. výpadek spojení s vysílačem. T případě chyby je třeba poslat nejdřív "e" a potom "P".

**Hlavní kód viz main.c ve složce Receiver.**
