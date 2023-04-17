This repo includes all the file's and arudino projects used to build an object recognition system using the ESP32-CAM with Edge Impulse. 

Two main scripts are included: 

	- Capture pics on loop, which get's new picutre and saves them to an SD card.
	
	- Main detection script which is the final running project. It takes picutres, classifys the images based on a
	dataset created and then output the image to a 160 * 128 ST7735 TFT display whith notes to display if any any objects
	have been recognised. 

The libraries included in the ESP32_Edge_Impulse.Ino script have some small changes made.

In order to run the script
The Libraries inluced here need to be copied to the arduino IDE libraries folder. 
The default folder is - C:\Users\"username"\Documents\Arduino\Libraries for windows

These libraries include: 
	- an edited TFT_eSPI folder that uses the ST7735 driver.
	- the edge impulse project export library for lego blocks
	- standard JPEGDecoder


Instuctional videos are availalbe to help with setting up edge impulse: 
https://www.youtube.com/watch?v=0oG6OF1SXto&t=149s&pp=ygUWZXNwMzItY2FtIGVkZ2UgaW1wdWxzZQ%3D%3D
