copy /y .\output\debug\E785.hex .\E785.hex
hex2pgx.exe --boardall --exec-8b 0x8785:E785.hex ssss.hex Head_E785.pgh
