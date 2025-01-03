
#
# cherryusb include
#
PROJ_CINCDIRS += platforms/component/cherryusb/common
PROJ_CINCDIRS += platforms/component/cherryusb/core
PROJ_CINCDIRS += platforms/component/cherryusb/class/cdc
PROJ_CINCDIRS += platforms/component/cherryusb/port/dwc2
PROJ_CINCDIRS += platforms/component/cherryusb/demo

#
# cherryusb source file
#
CSOURCES += platforms/component/cherryusb/core/usbd_core.c
CSOURCES += platforms/component/cherryusb/class/cdc/usbd_cdc_acm.c
CSOURCES += platforms/component/cherryusb/class/cdc/usbd_cdc_ecm.c
CSOURCES += platforms/component/cherryusb/port/dwc2/usb_dc_dwc2.c
CSOURCES += platforms/component/cherryusb/port/dwc2/usb_glue_st.c
CSOURCES += platforms/component/cherryusb/demo/cdc_acm_template.c
