# Prust-RTOS-Gen
New main repo for the C code that contains Prust module written in Rust. Most of it is generated code except the prust part. Auto-generated mostly for VST104

```
Prust-RTOS-Gen  
+-- Middlewares/Third_Party/  
|     +-- Prust                 <-submodule & link provided below
|     +-- Prust-RTOS            <-submodule & link provided below    
|     +-- Prust-Test            <-submodule & link provided below   
|     +-- FreeRTOS              <-FreeRTOS source code imported automatically by STM32CubeIDE
+-- Core                   <-main.c, stm32hal4xx.h etc.  
+-- Drivers                <-Drivers mostly auto-generated  
+-- Prust.ioc              <-STM32CubeIDE configuration  
...
```
You can check the submodules:  
- [Prust](https://github.com/visionspacetec/Prust): This module. PUS-C data structures.  
- [Prust-Test](https://github.com/visionspacetec/Prust-Test): To test the whole project.
- [Prust-RTOS-Gen](https://github.com/visionspacetec/Prust-RTOS-Gen): Complete project as Stm32CubeIde project.
- [Prust-RTOS](https://github.com/visionspacetec/Prust-RTOS): RTOS application part.

# How To Build  
```
git clone https://github.com/visionspacetec/Prust-RTOS-Gen.git
```
Check the requirements in [Prust-RTOS](https://github.com/visionspacetec/Prust-RTOS) then build with the following commands,
```
cd Middlewares/ThirdParty/Prust-RTOS
xargo build --target thumbv7em-none-eabihf 
```
You can now debug and run with Stm32CubeIDE's interface. 
For testing see [Prust-Test](https://github.com/visionspacetec/Prust-Test).

# Design
Design specific for the VST104 example:  
 
![Design](media/designvst.png)


# Short Tutorial Of This Demo 
- See how this was done from scratch in wiki page [here](https://github.com/visionspacetec/Prust/wiki/How-I-Built-This-On-VST104)
