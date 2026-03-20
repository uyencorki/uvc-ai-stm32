CMW_REL_DIR := Lib/Camera_Middleware
ISP_REL_DIR := $(CMW_REL_DIR)/ISP_Library

C_SOURCES_CMW += $(CMW_REL_DIR)/cmw_camera.c
C_SOURCES_CMW += $(CMW_REL_DIR)/cmw_utils.c
C_SOURCES_CMW += $(ISP_REL_DIR)/isp/Src/isp_algo.c
C_SOURCES_CMW += $(ISP_REL_DIR)/isp/Src/isp_awb_algo.c
C_SOURCES_CMW += $(ISP_REL_DIR)/isp/Src/isp_ae_algo.c
C_SOURCES_CMW += $(ISP_REL_DIR)/isp/Src/isp_cmd_parser.c
C_SOURCES_CMW += $(ISP_REL_DIR)/isp/Src/isp_core.c
C_SOURCES_CMW += $(ISP_REL_DIR)/isp/Src/isp_services.c
C_SOURCES_CMW += $(ISP_REL_DIR)/isp/Src/isp_tool_com.c
C_INCLUDES_CMW += -I$(CMW_REL_DIR)
C_INCLUDES_CMW += -I$(CMW_REL_DIR)/sensors

C_INCLUDES_CMW += -I$(ISP_REL_DIR)/isp/Inc
C_INCLUDES_CMW += -I$(ISP_REL_DIR)/evision/Inc

C_SOURCES += $(C_SOURCES_CMW)
C_INCLUDES += $(C_INCLUDES_CMW)
C_DEFS += $(C_DEFS_CMW)
