# AT32-ADC-Test-Arm-GCC

This code is written for the AT32F403RCT6, which is on a hoverboard mainboard.

It is flashed via SWD and a Segger J-Link mini edu.

As there is no direct flash access via SWD a flash loader has to be used. The appropriate flashloader was taken from the Keil uVision support package and is in a CMSIS Pack format. Using Segger OpenLoader feature one can add hos own support for the AT32F403.
For this add following code to your `<Segger Installation Dir>\JLinkDevices.xml`
    
    <!--                   -->
    <!-- Artery -->    
    <!--                   -->    
    <Device>        
    	<ChipInfo Vendor="Artery" Name="AT32F403RCT6" WorkRAMAddr="0x20000000" WorkRAMSize="0x38000" Core="JLINK_CORE_CORTEX_M4" />
	<FlashBankInfo Name="Internal Flash Bank 1" BaseAddr="0x08000000" MaxSize="0x80000" Loader="Devices/AT32F403_1024.FLM" LoaderType="FLASH_ALGO_TYPE_OPEN" />
    </Device>

and copy the "AT32F403_1024.FLM" from the folder flash to `<Segger Installation Dir>\Devices`
