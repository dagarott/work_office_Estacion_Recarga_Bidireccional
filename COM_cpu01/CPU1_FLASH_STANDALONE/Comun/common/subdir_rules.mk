################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
Comun/common/F2837xD_Adc.obj: C:/Users/dagaro/workspace/Firmware_Test/UFCharger/F2837xD/Comun/common/F2837xD_Adc.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/bin/cl2000" -v28 -ml -mt --vcu_support=vcu2 --tmu_support=tmu0 --cla_support=cla1 --float_support=fpu32 --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_common/include" -g --define=CPU1 --define=_STANDALONE --define=_FLASH --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="Comun/common/$(basename $(<F)).d_raw" --obj_directory="Comun/common" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Comun/common/F2837xD_CodeStartBranch.obj: C:/Users/dagaro/workspace/Firmware_Test/UFCharger/F2837xD/Comun/common/F2837xD_CodeStartBranch.asm $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/bin/cl2000" -v28 -ml -mt --vcu_support=vcu2 --tmu_support=tmu0 --cla_support=cla1 --float_support=fpu32 --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_common/include" -g --define=CPU1 --define=_STANDALONE --define=_FLASH --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="Comun/common/$(basename $(<F)).d_raw" --obj_directory="Comun/common" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Comun/common/F2837xD_CpuTimers.obj: C:/Users/dagaro/workspace/Firmware_Test/UFCharger/F2837xD/Comun/common/F2837xD_CpuTimers.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/bin/cl2000" -v28 -ml -mt --vcu_support=vcu2 --tmu_support=tmu0 --cla_support=cla1 --float_support=fpu32 --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_common/include" -g --define=CPU1 --define=_STANDALONE --define=_FLASH --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="Comun/common/$(basename $(<F)).d_raw" --obj_directory="Comun/common" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Comun/common/F2837xD_DBGIER.obj: C:/Users/dagaro/workspace/Firmware_Test/UFCharger/F2837xD/Comun/common/F2837xD_DBGIER.asm $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/bin/cl2000" -v28 -ml -mt --vcu_support=vcu2 --tmu_support=tmu0 --cla_support=cla1 --float_support=fpu32 --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_common/include" -g --define=CPU1 --define=_STANDALONE --define=_FLASH --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="Comun/common/$(basename $(<F)).d_raw" --obj_directory="Comun/common" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Comun/common/F2837xD_DefaultISR.obj: C:/Users/dagaro/workspace/Firmware_Test/UFCharger/F2837xD/Comun/common/F2837xD_DefaultISR.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/bin/cl2000" -v28 -ml -mt --vcu_support=vcu2 --tmu_support=tmu0 --cla_support=cla1 --float_support=fpu32 --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_common/include" -g --define=CPU1 --define=_STANDALONE --define=_FLASH --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="Comun/common/$(basename $(<F)).d_raw" --obj_directory="Comun/common" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Comun/common/F2837xD_Dma.obj: C:/Users/dagaro/workspace/Firmware_Test/UFCharger/F2837xD/Comun/common/F2837xD_Dma.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/bin/cl2000" -v28 -ml -mt --vcu_support=vcu2 --tmu_support=tmu0 --cla_support=cla1 --float_support=fpu32 --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_common/include" -g --define=CPU1 --define=_STANDALONE --define=_FLASH --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="Comun/common/$(basename $(<F)).d_raw" --obj_directory="Comun/common" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Comun/common/F2837xD_ECap.obj: C:/Users/dagaro/workspace/Firmware_Test/UFCharger/F2837xD/Comun/common/F2837xD_ECap.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/bin/cl2000" -v28 -ml -mt --vcu_support=vcu2 --tmu_support=tmu0 --cla_support=cla1 --float_support=fpu32 --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_common/include" -g --define=CPU1 --define=_STANDALONE --define=_FLASH --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="Comun/common/$(basename $(<F)).d_raw" --obj_directory="Comun/common" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Comun/common/F2837xD_EPwm.obj: C:/Users/dagaro/workspace/Firmware_Test/UFCharger/F2837xD/Comun/common/F2837xD_EPwm.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/bin/cl2000" -v28 -ml -mt --vcu_support=vcu2 --tmu_support=tmu0 --cla_support=cla1 --float_support=fpu32 --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_common/include" -g --define=CPU1 --define=_STANDALONE --define=_FLASH --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="Comun/common/$(basename $(<F)).d_raw" --obj_directory="Comun/common" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Comun/common/F2837xD_EQep.obj: C:/Users/dagaro/workspace/Firmware_Test/UFCharger/F2837xD/Comun/common/F2837xD_EQep.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/bin/cl2000" -v28 -ml -mt --vcu_support=vcu2 --tmu_support=tmu0 --cla_support=cla1 --float_support=fpu32 --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_common/include" -g --define=CPU1 --define=_STANDALONE --define=_FLASH --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="Comun/common/$(basename $(<F)).d_raw" --obj_directory="Comun/common" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Comun/common/F2837xD_Emif.obj: C:/Users/dagaro/workspace/Firmware_Test/UFCharger/F2837xD/Comun/common/F2837xD_Emif.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/bin/cl2000" -v28 -ml -mt --vcu_support=vcu2 --tmu_support=tmu0 --cla_support=cla1 --float_support=fpu32 --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_common/include" -g --define=CPU1 --define=_STANDALONE --define=_FLASH --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="Comun/common/$(basename $(<F)).d_raw" --obj_directory="Comun/common" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Comun/common/F2837xD_GlobalVariableDefs.obj: C:/Users/dagaro/workspace/Firmware_Test/UFCharger/F2837xD/Comun/common/F2837xD_GlobalVariableDefs.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/bin/cl2000" -v28 -ml -mt --vcu_support=vcu2 --tmu_support=tmu0 --cla_support=cla1 --float_support=fpu32 --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_common/include" -g --define=CPU1 --define=_STANDALONE --define=_FLASH --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="Comun/common/$(basename $(<F)).d_raw" --obj_directory="Comun/common" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Comun/common/F2837xD_Gpio.obj: C:/Users/dagaro/workspace/Firmware_Test/UFCharger/F2837xD/Comun/common/F2837xD_Gpio.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/bin/cl2000" -v28 -ml -mt --vcu_support=vcu2 --tmu_support=tmu0 --cla_support=cla1 --float_support=fpu32 --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_common/include" -g --define=CPU1 --define=_STANDALONE --define=_FLASH --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="Comun/common/$(basename $(<F)).d_raw" --obj_directory="Comun/common" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Comun/common/F2837xD_I2C.obj: C:/Users/dagaro/workspace/Firmware_Test/UFCharger/F2837xD/Comun/common/F2837xD_I2C.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/bin/cl2000" -v28 -ml -mt --vcu_support=vcu2 --tmu_support=tmu0 --cla_support=cla1 --float_support=fpu32 --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_common/include" -g --define=CPU1 --define=_STANDALONE --define=_FLASH --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="Comun/common/$(basename $(<F)).d_raw" --obj_directory="Comun/common" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Comun/common/F2837xD_Ipc.obj: C:/Users/dagaro/workspace/Firmware_Test/UFCharger/F2837xD/Comun/common/F2837xD_Ipc.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/bin/cl2000" -v28 -ml -mt --vcu_support=vcu2 --tmu_support=tmu0 --cla_support=cla1 --float_support=fpu32 --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_common/include" -g --define=CPU1 --define=_STANDALONE --define=_FLASH --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="Comun/common/$(basename $(<F)).d_raw" --obj_directory="Comun/common" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Comun/common/F2837xD_Ipc_Driver.obj: C:/Users/dagaro/workspace/Firmware_Test/UFCharger/F2837xD/Comun/common/F2837xD_Ipc_Driver.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/bin/cl2000" -v28 -ml -mt --vcu_support=vcu2 --tmu_support=tmu0 --cla_support=cla1 --float_support=fpu32 --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_common/include" -g --define=CPU1 --define=_STANDALONE --define=_FLASH --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="Comun/common/$(basename $(<F)).d_raw" --obj_directory="Comun/common" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Comun/common/F2837xD_Ipc_Driver_Lite.obj: C:/Users/dagaro/workspace/Firmware_Test/UFCharger/F2837xD/Comun/common/F2837xD_Ipc_Driver_Lite.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/bin/cl2000" -v28 -ml -mt --vcu_support=vcu2 --tmu_support=tmu0 --cla_support=cla1 --float_support=fpu32 --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_common/include" -g --define=CPU1 --define=_STANDALONE --define=_FLASH --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="Comun/common/$(basename $(<F)).d_raw" --obj_directory="Comun/common" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Comun/common/F2837xD_Ipc_Driver_Util.obj: C:/Users/dagaro/workspace/Firmware_Test/UFCharger/F2837xD/Comun/common/F2837xD_Ipc_Driver_Util.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/bin/cl2000" -v28 -ml -mt --vcu_support=vcu2 --tmu_support=tmu0 --cla_support=cla1 --float_support=fpu32 --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_common/include" -g --define=CPU1 --define=_STANDALONE --define=_FLASH --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="Comun/common/$(basename $(<F)).d_raw" --obj_directory="Comun/common" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Comun/common/F2837xD_Mcbsp.obj: C:/Users/dagaro/workspace/Firmware_Test/UFCharger/F2837xD/Comun/common/F2837xD_Mcbsp.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/bin/cl2000" -v28 -ml -mt --vcu_support=vcu2 --tmu_support=tmu0 --cla_support=cla1 --float_support=fpu32 --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_common/include" -g --define=CPU1 --define=_STANDALONE --define=_FLASH --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="Comun/common/$(basename $(<F)).d_raw" --obj_directory="Comun/common" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Comun/common/F2837xD_PieCtrl.obj: C:/Users/dagaro/workspace/Firmware_Test/UFCharger/F2837xD/Comun/common/F2837xD_PieCtrl.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/bin/cl2000" -v28 -ml -mt --vcu_support=vcu2 --tmu_support=tmu0 --cla_support=cla1 --float_support=fpu32 --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_common/include" -g --define=CPU1 --define=_STANDALONE --define=_FLASH --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="Comun/common/$(basename $(<F)).d_raw" --obj_directory="Comun/common" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Comun/common/F2837xD_PieVect.obj: C:/Users/dagaro/workspace/Firmware_Test/UFCharger/F2837xD/Comun/common/F2837xD_PieVect.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/bin/cl2000" -v28 -ml -mt --vcu_support=vcu2 --tmu_support=tmu0 --cla_support=cla1 --float_support=fpu32 --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_common/include" -g --define=CPU1 --define=_STANDALONE --define=_FLASH --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="Comun/common/$(basename $(<F)).d_raw" --obj_directory="Comun/common" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Comun/common/F2837xD_Sci.obj: C:/Users/dagaro/workspace/Firmware_Test/UFCharger/F2837xD/Comun/common/F2837xD_Sci.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/bin/cl2000" -v28 -ml -mt --vcu_support=vcu2 --tmu_support=tmu0 --cla_support=cla1 --float_support=fpu32 --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_common/include" -g --define=CPU1 --define=_STANDALONE --define=_FLASH --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="Comun/common/$(basename $(<F)).d_raw" --obj_directory="Comun/common" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Comun/common/F2837xD_Spi.obj: C:/Users/dagaro/workspace/Firmware_Test/UFCharger/F2837xD/Comun/common/F2837xD_Spi.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/bin/cl2000" -v28 -ml -mt --vcu_support=vcu2 --tmu_support=tmu0 --cla_support=cla1 --float_support=fpu32 --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_common/include" -g --define=CPU1 --define=_STANDALONE --define=_FLASH --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="Comun/common/$(basename $(<F)).d_raw" --obj_directory="Comun/common" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Comun/common/F2837xD_SysCtrl.obj: C:/Users/dagaro/workspace/Firmware_Test/UFCharger/F2837xD/Comun/common/F2837xD_SysCtrl.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/bin/cl2000" -v28 -ml -mt --vcu_support=vcu2 --tmu_support=tmu0 --cla_support=cla1 --float_support=fpu32 --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_common/include" -g --define=CPU1 --define=_STANDALONE --define=_FLASH --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="Comun/common/$(basename $(<F)).d_raw" --obj_directory="Comun/common" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Comun/common/F2837xD_TempSensorConv.obj: C:/Users/dagaro/workspace/Firmware_Test/UFCharger/F2837xD/Comun/common/F2837xD_TempSensorConv.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/bin/cl2000" -v28 -ml -mt --vcu_support=vcu2 --tmu_support=tmu0 --cla_support=cla1 --float_support=fpu32 --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_common/include" -g --define=CPU1 --define=_STANDALONE --define=_FLASH --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="Comun/common/$(basename $(<F)).d_raw" --obj_directory="Comun/common" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Comun/common/F2837xD_Upp.obj: C:/Users/dagaro/workspace/Firmware_Test/UFCharger/F2837xD/Comun/common/F2837xD_Upp.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/bin/cl2000" -v28 -ml -mt --vcu_support=vcu2 --tmu_support=tmu0 --cla_support=cla1 --float_support=fpu32 --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_common/include" -g --define=CPU1 --define=_STANDALONE --define=_FLASH --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="Comun/common/$(basename $(<F)).d_raw" --obj_directory="Comun/common" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Comun/common/F2837xD_can.obj: C:/Users/dagaro/workspace/Firmware_Test/UFCharger/F2837xD/Comun/common/F2837xD_can.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/bin/cl2000" -v28 -ml -mt --vcu_support=vcu2 --tmu_support=tmu0 --cla_support=cla1 --float_support=fpu32 --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_common/include" -g --define=CPU1 --define=_STANDALONE --define=_FLASH --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="Comun/common/$(basename $(<F)).d_raw" --obj_directory="Comun/common" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Comun/common/F2837xD_sci_io.obj: C:/Users/dagaro/workspace/Firmware_Test/UFCharger/F2837xD/Comun/common/F2837xD_sci_io.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/bin/cl2000" -v28 -ml -mt --vcu_support=vcu2 --tmu_support=tmu0 --cla_support=cla1 --float_support=fpu32 --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_common/include" -g --define=CPU1 --define=_STANDALONE --define=_FLASH --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="Comun/common/$(basename $(<F)).d_raw" --obj_directory="Comun/common" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Comun/common/F2837xD_sdfm_drivers.obj: C:/Users/dagaro/workspace/Firmware_Test/UFCharger/F2837xD/Comun/common/F2837xD_sdfm_drivers.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/bin/cl2000" -v28 -ml -mt --vcu_support=vcu2 --tmu_support=tmu0 --cla_support=cla1 --float_support=fpu32 --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_common/include" -g --define=CPU1 --define=_STANDALONE --define=_FLASH --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="Comun/common/$(basename $(<F)).d_raw" --obj_directory="Comun/common" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Comun/common/F2837xD_struct.obj: C:/Users/dagaro/workspace/Firmware_Test/UFCharger/F2837xD/Comun/common/F2837xD_struct.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/bin/cl2000" -v28 -ml -mt --vcu_support=vcu2 --tmu_support=tmu0 --cla_support=cla1 --float_support=fpu32 --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_common/include" -g --define=CPU1 --define=_STANDALONE --define=_FLASH --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="Comun/common/$(basename $(<F)).d_raw" --obj_directory="Comun/common" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Comun/common/F2837xD_usDelay.obj: C:/Users/dagaro/workspace/Firmware_Test/UFCharger/F2837xD/Comun/common/F2837xD_usDelay.asm $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/bin/cl2000" -v28 -ml -mt --vcu_support=vcu2 --tmu_support=tmu0 --cla_support=cla1 --float_support=fpu32 --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-c2000_18.1.3.LTS/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xD/v210/F2837xD_common/include" -g --define=CPU1 --define=_STANDALONE --define=_FLASH --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="Comun/common/$(basename $(<F)).d_raw" --obj_directory="Comun/common" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

