################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
Comun/driverlib/can.obj: C:/Users/dagaro/workspace/Firmware_Test/UFCharger/F2837xD/Comun/driverlib/can.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/bin/cl2000" -v28 -ml -mt --vcu_support=vcu2 --tmu_support=tmu0 --cla_support=cla1 --float_support=fpu32 --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_common/include" -g --define=CPU1 --define=_STANDALONE --define=_FLASH --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="Comun/driverlib/$(basename $(<F)).d_raw" --obj_directory="Comun/driverlib" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Comun/driverlib/interrupt.obj: C:/Users/dagaro/workspace/Firmware_Test/UFCharger/F2837xD/Comun/driverlib/interrupt.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/bin/cl2000" -v28 -ml -mt --vcu_support=vcu2 --tmu_support=tmu0 --cla_support=cla1 --float_support=fpu32 --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_common/include" -g --define=CPU1 --define=_STANDALONE --define=_FLASH --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="Comun/driverlib/$(basename $(<F)).d_raw" --obj_directory="Comun/driverlib" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Comun/driverlib/sysctl.obj: C:/Users/dagaro/workspace/Firmware_Test/UFCharger/F2837xD/Comun/driverlib/sysctl.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/bin/cl2000" -v28 -ml -mt --vcu_support=vcu2 --tmu_support=tmu0 --cla_support=cla1 --float_support=fpu32 --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_common/include" -g --define=CPU1 --define=_STANDALONE --define=_FLASH --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="Comun/driverlib/$(basename $(<F)).d_raw" --obj_directory="Comun/driverlib" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Comun/driverlib/systick.obj: C:/Users/dagaro/workspace/Firmware_Test/UFCharger/F2837xD/Comun/driverlib/systick.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/bin/cl2000" -v28 -ml -mt --vcu_support=vcu2 --tmu_support=tmu0 --cla_support=cla1 --float_support=fpu32 --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_common/include" -g --define=CPU1 --define=_STANDALONE --define=_FLASH --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="Comun/driverlib/$(basename $(<F)).d_raw" --obj_directory="Comun/driverlib" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Comun/driverlib/uart.obj: C:/Users/dagaro/workspace/Firmware_Test/UFCharger/F2837xD/Comun/driverlib/uart.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/bin/cl2000" -v28 -ml -mt --vcu_support=vcu2 --tmu_support=tmu0 --cla_support=cla1 --float_support=fpu32 --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_common/include" -g --define=CPU1 --define=_STANDALONE --define=_FLASH --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="Comun/driverlib/$(basename $(<F)).d_raw" --obj_directory="Comun/driverlib" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Comun/driverlib/usb.obj: C:/Users/dagaro/workspace/Firmware_Test/UFCharger/F2837xD/Comun/driverlib/usb.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/bin/cl2000" -v28 -ml -mt --vcu_support=vcu2 --tmu_support=tmu0 --cla_support=cla1 --float_support=fpu32 --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_common/include" -g --define=CPU1 --define=_STANDALONE --define=_FLASH --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="Comun/driverlib/$(basename $(<F)).d_raw" --obj_directory="Comun/driverlib" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Comun/driverlib/usb_hal.obj: C:/Users/dagaro/workspace/Firmware_Test/UFCharger/F2837xD/Comun/driverlib/usb_hal.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/bin/cl2000" -v28 -ml -mt --vcu_support=vcu2 --tmu_support=tmu0 --cla_support=cla1 --float_support=fpu32 --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_common/include" -g --define=CPU1 --define=_STANDALONE --define=_FLASH --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="Comun/driverlib/$(basename $(<F)).d_raw" --obj_directory="Comun/driverlib" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


