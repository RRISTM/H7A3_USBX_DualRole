# H7A3_USBX_DualRole

Example of dual role USBX application. 

Main part in appusbx_host.c in function usbx_app_thread_entry

There is state machine with six states. 

Device init
Device process
Device deinit
Host init 
Host process
Host deinit

The switch between device and host is done by checking pin state. In this case PA10. 
The state value is checked in host process and device process. 

The host currently is HID host waiting for mouse. 
Device is CDC with loopback 



