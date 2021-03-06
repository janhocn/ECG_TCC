#include <stdlib.h>

#include "GUI.h"

#ifndef GUI_CONST_STORAGE
#define GUI_CONST_STORAGE const
#endif

static GUI_CONST_STORAGE unsigned long _acSPO2ValueIcon[] = { 0xFF000000, 0xFF000000, 0xFF000000, 0xFF000000,
        0xC226224C, 0x5E5B53B4, 0x1E796EEE, 0x048276FF, 0x008276FF, 0x008276FF, 0x008276FF, 0x008276FF, 0x008276FF,
        0x008276FF, 0x008276FF, 0x008276FF, 0x008276FF, 0x008276FF, 0x008276FF, 0x008276FF, 0x008276FF, 0x008276FF,
        0x008276FF, 0x008276FF, 0x008276FF, 0x008276FF, 0x008276FF, 0x008276FF, 0x008276FF, 0x008276FF, 0x008276FF,
        0x008276FF, 0x008276FF, 0x008276FF, 0x008276FF, 0x008276FF, 0x008276FF, 0x048276FF, 0x1E796EEE, 0x5E5B53B4,
        0xC226224C, 0xFF000000, 0xFF000000, 0xFF000000, 0xFF000000, 0xFF000000, 0xFF000000, 0xEF0B0A17, 0x575A52B3,
        0x018175FE, 0x008175FF, 0x008176FF, 0x008176FF, 0x008175FF, 0x008175FF, 0x008175FF, 0x008176FF, 0x008176FF,
        0x008176FF, 0x008175FF, 0x008175FF, 0x008176FF, 0x008175FF, 0x008176FF, 0x008175FF, 0x008176FF, 0x008175FF,
        0x008175FF, 0x008175FF, 0x008175FF, 0x008175FF, 0x008175FF, 0x008176FF, 0x008176FF, 0x008176FF, 0x008175FF,
        0x008176FF, 0x008175FF, 0x008175FF, 0x008175FF, 0x008176FF, 0x008175FF, 0x008176FF, 0x008175FF, 0x008175FF,
        0x018175FE, 0x575A52B3, 0xEF0B0A17, 0xFF000000, 0xFF000000, 0xFF000000, 0xEF0B0A17, 0x336B62D4, 0x008075FF,
        0x008075FF, 0x008075FF, 0x008075FF, 0x008075FF, 0x008075FF, 0x008075FF, 0x008075FF, 0x008075FF, 0x008075FF,
        0x008075FF, 0x008075FF, 0x008075FF, 0x008075FF, 0x008075FF, 0x008075FF, 0x008075FF, 0x008075FF, 0x008075FF,
        0x008075FF, 0x008075FF, 0x008075FF, 0x008075FF, 0x008075FF, 0x008075FF, 0x008075FF, 0x008075FF, 0x008075FF,
        0x008075FF, 0x008075FF, 0x008075FF, 0x008075FF, 0x008075FF, 0x008075FF, 0x008075FF, 0x008075FF, 0x008075FF,
        0x008075FF, 0x008075FF, 0x336B62D4, 0xEF0B0A17, 0xFF000000, 0xFF000000, 0x576057B3, 0x008075FF, 0x008075FF,
        0x008075FF, 0x008075FF, 0x008075FF, 0x008075FF, 0x008075FF, 0x008075FF, 0x008075FF, 0x008075FF, 0x008075FF,
        0x008075FF, 0x008075FF, 0x008075FF, 0x008075FF, 0x008075FF, 0x008075FF, 0x008075FF, 0x008075FF, 0x008075FF,
        0x008075FF, 0x008075FF, 0x008074FF, 0x008075FF, 0x008075FF, 0x008074FF, 0x008075FF, 0x008075FF, 0x008075FF,
        0x008074FF, 0x008075FF, 0x008074FF, 0x008075FF, 0x008074FF, 0x008074FF, 0x008075FF, 0x008075FF, 0x008074FF,
        0x008075FF, 0x008075FF, 0x008075FF, 0x575C57B3, 0xFF000000, 0xC225224C, 0x017F74FE, 0x008074FF, 0x007F74FF,
        0x008074FF, 0x007F74FF, 0x008074FF, 0x008074FF, 0x008074FF, 0x008074FF, 0x008074FF, 0x007F74FF, 0x008074FF,
        0x008074FF, 0x008074FF, 0x008074FF, 0x008074FF, 0x008074FF, 0x008074FF, 0x007F74FF, 0x008074FF, 0x007F74FF,
        0x008074FF, 0x008074FF, 0x008074FF, 0x008074FF, 0x008074FF, 0x008074FF, 0x008074FF, 0x008074FF, 0x008074FF,
        0x008074FF, 0x008074FF, 0x007F74FF, 0x008074FF, 0x008074FF, 0x008074FF, 0x008074FF, 0x008074FF, 0x008074FF,
        0x008074FF, 0x008074FF, 0x008074FF, 0x017F74FE, 0xC225224C, 0x5E5951B5, 0x007F74FF, 0x007F74FF, 0x007F73FF,
        0x007F73FF, 0x007F74FF, 0x007F74FF, 0x007F74FF, 0x007F73FF, 0x007F74FF, 0x007F74FF, 0x007F74FF, 0x007F74FF,
        0x007F74FF, 0x007F74FF, 0x007F74FF, 0x007F74FF, 0x007F74FF, 0x007F74FF, 0x007F74FF, 0x007F74FF, 0x007F74FF,
        0x007F74FF, 0x007F73FF, 0x007F74FF, 0x007F74FF, 0x007F74FF, 0x007F74FF, 0x007F74FF, 0x007F74FF, 0x007F74FF,
        0x007F74FF, 0x007F73FF, 0x007F74FF, 0x007F74FF, 0x007F73FF, 0x007F74FF, 0x007F74FF, 0x007F74FF, 0x007F74FF,
        0x007F74FF, 0x007F74FF, 0x007F74FF, 0x007F74FF, 0x5E5951B5, 0x1E766BEE, 0x007E73FF, 0x007F73FF, 0x007F73FF,
        0x007E73FF, 0x007F73FF, 0x007F73FF, 0x007F73FF, 0x007F73FF, 0x007F73FF, 0x007F73FF, 0x007E73FF, 0x007F73FF,
        0x007F73FF, 0x007F73FF, 0x007F73FF, 0x008E84FB, 0x009990F9, 0x008074FF, 0x007F73FF, 0x007F73FF, 0x007F73FF,
        0x007F73FF, 0x007F73FF, 0x007F73FF, 0x007F73FF, 0x007F73FF, 0x007F73FF, 0x007F73FF, 0x007F73FF, 0x007F73FF,
        0x007F73FF, 0x007F73FF, 0x007F73FF, 0x007F73FF, 0x007F73FF, 0x007F73FF, 0x007F73FF, 0x007F73FF, 0x007F73FF,
        0x007F73FF, 0x007F73FF, 0x007F73FF, 0x007F73FF, 0x1E766BEE, 0x047E72FF, 0x007E72FF, 0x007E73FF, 0x007E72FF,
        0x007E73FF, 0x007E72FF, 0x007E72FF, 0x007E72FF, 0x007E72FF, 0x007E72FF, 0x007E73FF, 0x007E73FF, 0x007E72FF,
        0x007E73FF, 0x007E73FF, 0x00A8A1F6, 0x00FAFAE7, 0x00FCFCE7, 0x00D7D5ED, 0x007F74FF, 0x007E72FF, 0x007E72FF,
        0x007E72FF, 0x007E72FF, 0x007E73FF, 0x007E72FF, 0x007E73FF, 0x007E72FF, 0x007E72FF, 0x007E72FF, 0x007E72FF,
        0x007E72FF, 0x007E72FF, 0x007E73FF, 0x007E73FF, 0x007E72FF, 0x007E72FF, 0x007E72FF, 0x007E72FF, 0x007E73FF,
        0x007E73FF, 0x007E72FF, 0x007E72FF, 0x007E72FF, 0x047E72FF, 0x007E72FF, 0x007E72FF, 0x007E72FF, 0x007E72FF,
        0x007E72FF, 0x007E72FF, 0x007E72FF, 0x007E72FF, 0x007E72FF, 0x007E72FF, 0x007E72FF, 0x007E72FF, 0x007E72FF,
        0x007E72FF, 0x008B81FC, 0x00F5F4E8, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00B6B1F3, 0x007E72FF, 0x007E72FF,
        0x007E72FF, 0x007E72FF, 0x007E72FF, 0x007E72FF, 0x007E72FF, 0x007E72FF, 0x007E72FF, 0x007E72FF, 0x007E72FF,
        0x007E72FF, 0x007E72FF, 0x007E72FF, 0x007E72FF, 0x007E72FF, 0x007E72FF, 0x007E72FF, 0x007E72FF, 0x007E72FF,
        0x007E72FF, 0x007E72FF, 0x007E72FF, 0x007E72FF, 0x007E72FF, 0x007D71FF, 0x007D71FF, 0x007D71FF, 0x007D71FF,
        0x007D71FF, 0x007D71FF, 0x007D71FF, 0x007D71FF, 0x007D71FF, 0x007D71FF, 0x007D71FF, 0x007D71FF, 0x007D71FF,
        0x007E72FF, 0x00DAD7ED, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00F8F8E8, 0x009288FB, 0x007D71FF,
        0x007D71FF, 0x007D71FF, 0x007D71FF, 0x007D71FF, 0x007D71FF, 0x007D71FF, 0x007D71FF, 0x007D71FF, 0x007D71FF,
        0x007D71FF, 0x007D71FF, 0x007D71FF, 0x007D71FF, 0x007D71FF, 0x007D71FF, 0x007D71FF, 0x007D71FF, 0x007D71FF,
        0x007D71FF, 0x007D71FF, 0x007D71FF, 0x007D71FF, 0x007D71FF, 0x007D71FF, 0x007D71FF, 0x007D71FF, 0x007D71FF,
        0x007D70FF, 0x007D70FF, 0x007D70FF, 0x007D70FF, 0x007D71FF, 0x007D70FF, 0x007D71FF, 0x007D70FF, 0x007D71FF,
        0x00B0A9F5, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00E1E0EC, 0x007D72FF,
        0x007D71FF, 0x007D71FF, 0x007D70FF, 0x007D70FF, 0x007D70FF, 0x007D71FF, 0x007D71FF, 0x007D71FF, 0x007D71FF,
        0x007D71FF, 0x007D70FF, 0x007D71FF, 0x007D71FF, 0x007D71FF, 0x007D71FF, 0x007D71FF, 0x007D71FF, 0x007D71FF,
        0x007D70FF, 0x007D71FF, 0x007D71FF, 0x007D71FF, 0x007D71FF, 0x007C70FF, 0x007C70FF, 0x007C70FF, 0x007C70FF,
        0x007C70FF, 0x007C70FF, 0x007C70FF, 0x007C70FF, 0x007C70FF, 0x007C70FF, 0x007C70FF, 0x007C70FF, 0x008A7FFC,
        0x00F5F4E8, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00B5B0F4,
        0x007C70FF, 0x007C70FF, 0x007C70FF, 0x007C70FF, 0x007C70FF, 0x007C70FF, 0x007C70FF, 0x007C70FF, 0x007C70FF,
        0x007C70FF, 0x007C70FF, 0x007C70FF, 0x007C70FF, 0x007C70FF, 0x007C70FF, 0x007C70FF, 0x007C70FF, 0x007C70FF,
        0x007C70FF, 0x007C70FF, 0x007C70FF, 0x007C70FF, 0x007C70FF, 0x007B6FFF, 0x007C6FFF, 0x007C6FFF, 0x007C6FFF,
        0x007C6FFF, 0x007B6FFF, 0x007B6FFF, 0x007C6FFF, 0x007B6FFF, 0x007C6FFF, 0x007B6FFF, 0x007B6FFF, 0x00D4D1EE,
        0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00F7F7E8,
        0x008B81FB, 0x007B6FFF, 0x007C6FFF, 0x007C6FFF, 0x007B6FFF, 0x007B6FFF, 0x007C6FFF, 0x007B6FFF, 0x007B6FFF,
        0x007B6FFF, 0x007C6FFF, 0x007B6FFF, 0x007C6FFF, 0x007C6FFF, 0x007C6FFF, 0x007B6FFF, 0x007B6FFF, 0x007B6FFF,
        0x007C6FFF, 0x007B6FFF, 0x007B6FFF, 0x007B6FFF, 0x007C6FFF, 0x007A6EFF, 0x007A6EFF, 0x007A6EFF, 0x007A6EFF,
        0x007A6EFF, 0x007A6EFF, 0x007A6EFF, 0x007A6EFF, 0x007A6EFF, 0x007A6EFF, 0x007A6EFF, 0x00A29AF7, 0x00FCFCE7,
        0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7,
        0x00D6D3ED, 0x007A6EFF, 0x007A6EFF, 0x007A6EFF, 0x007A6EFF, 0x007A6EFF, 0x007A6EFF, 0x007A6EFF, 0x007A6FFF,
        0x007A6EFF, 0x007A6EFF, 0x007A6EFF, 0x007A6EFF, 0x007A6EFF, 0x007A6EFF, 0x007A6EFF, 0x007A6EFF, 0x007A6EFF,
        0x007B6FFF, 0x007A6EFF, 0x007A6EFF, 0x007A6EFF, 0x007B6EFF, 0x007A6DFF, 0x007A6EFF, 0x007A6EFF, 0x007A6EFF,
        0x007A6EFF, 0x007A6DFF, 0x007A6EFF, 0x007A6EFF, 0x007A6EFF, 0x007A6EFF, 0x007E72FE, 0x00EBEAEA, 0x00FCFCE7,
        0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7,
        0x00FCFCE7, 0x00A199F7, 0x007A6EFF, 0x007A6EFF, 0x007A6EFF, 0x007A6DFF, 0x007A6EFF, 0x007A6DFF, 0x007A6DFF,
        0x007A6DFF, 0x007A6EFF, 0x007A6EFF, 0x007A6EFF, 0x007A6EFF, 0x007A6EFF, 0x007A6EFF, 0x007A6EFF, 0x007A6DFF,
        0x007A6EFF, 0x007A6EFF, 0x007A6EFF, 0x007A6EFF, 0x007A6EFF, 0x00796DFF, 0x00796DFF, 0x00796DFF, 0x00796DFF,
        0x00796DFF, 0x007A6DFF, 0x00796DFF, 0x00796DFF, 0x00796DFF, 0x00796DFF, 0x00B8B3F3, 0x00FCFCE7, 0x00FCFCE7,
        0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7,
        0x00F8F8E8, 0x00D0CDEE, 0x00786BFF, 0x00796DFF, 0x00796DFF, 0x00796DFF, 0x00796DFF, 0x00796DFF, 0x00796DFF,
        0x00796DFF, 0x00796DFF, 0x00796DFF, 0x00796DFF, 0x00796DFF, 0x00796DFF, 0x00796DFF, 0x00796DFF, 0x00796DFF,
        0x00796DFF, 0x00796DFF, 0x007A6DFF, 0x00796DFF, 0x00796DFF, 0x00796CFF, 0x00796CFF, 0x00796CFF, 0x00796CFF,
        0x00796CFF, 0x00796CFF, 0x00796CFF, 0x00796CFF, 0x00796CFF, 0x008379FD, 0x00F5F5E8, 0x00FCFCE7, 0x00FCFCE7,
        0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00DFDDEB, 0x00A099F6,
        0x007A6FFD, 0x007569FF, 0x00796CFF, 0x00796CFF, 0x00796CFF, 0x00796CFF, 0x00796CFF, 0x00796CFF, 0x00796CFF,
        0x00796CFF, 0x00796CFF, 0x00796CFF, 0x00796CFF, 0x00796CFF, 0x00796CFF, 0x00796CFF, 0x00796CFF, 0x00796CFF,
        0x00796CFF, 0x00796CFF, 0x00796CFF, 0x00796CFF, 0x00796CFF, 0x00776BFF, 0x00776BFF, 0x00776BFF, 0x00776BFF,
        0x00786BFF, 0x00786BFF, 0x00776BFF, 0x00776BFF, 0x00786BFF, 0x00C1BDF1, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7,
        0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00F7F7E8, 0x00A7A1F5, 0x007469FF, 0x00776AFF,
        0x00867BFC, 0x00A8A1F6, 0x00BBB5F2, 0x00BCB6F2, 0x00ACA5F5, 0x008B81FB, 0x00776AFF, 0x00786BFF, 0x00786BFF,
        0x00786BFF, 0x00776BFF, 0x00786BFF, 0x00786BFF, 0x00786BFF, 0x00786BFF, 0x00786BFF, 0x00776BFF, 0x00786BFF,
        0x00786BFF, 0x00776BFF, 0x00776BFF, 0x00786BFF, 0x00786BFF, 0x00776AFF, 0x00776AFF, 0x00776AFF, 0x00776AFF,
        0x00776AFF, 0x00776AFF, 0x00776AFF, 0x00776AFF, 0x008277FD, 0x00F8F8E8, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7,
        0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00F5F5E8, 0x008E86F9, 0x00766AFF, 0x008A81FB, 0x00D4D1EE,
        0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00DCDAEC, 0x00928AF9, 0x00776AFF,
        0x00776AFF, 0x00776AFF, 0x00776AFF, 0x00776AFF, 0x00776AFF, 0x00776AFF, 0x00776AFF, 0x00776AFF, 0x00776AFF,
        0x00776AFF, 0x00776AFF, 0x00776AFF, 0x00776AFF, 0x00776AFF, 0x007669FF, 0x00766AFF, 0x007669FF, 0x00766AFF,
        0x00766AFF, 0x00766AFF, 0x00766AFF, 0x00766AFF, 0x00BAB5F2, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7,
        0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FBFBE7, 0x00968EF7, 0x007669FF, 0x00A098F7, 0x00F6F6E8, 0x00FCFCE7,
        0x00FCFCE7, 0x00FCFCE7, 0x00F7F7E8, 0x00F6F6E8, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FAFAE7, 0x00ADA7F4,
        0x007669FF, 0x007669FF, 0x00766AFF, 0x00766AFF, 0x00766AFF, 0x00766AFF, 0x00766AFF, 0x00766AFF, 0x00766AFF,
        0x00766AFF, 0x00766AFF, 0x00766AFF, 0x007669FF, 0x00766AFF, 0x007669FF, 0x007669FF, 0x007669FF, 0x007669FF,
        0x007669FF, 0x007669FF, 0x007569FF, 0x00786CFE, 0x00EFEFE9, 0x00E9E8EA, 0x00B5B0F2, 0x00E4E3EB, 0x00FCFCE7,
        0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00C1BDF0, 0x007467FF, 0x00978EF8, 0x00F9F9E7, 0x00FCFCE7, 0x00FBFBE7,
        0x00BDB9F1, 0x00877FFA, 0x007267FE, 0x007266FE, 0x00857BFB, 0x00B7B2F2, 0x00F7F7E8, 0x00FCFCE7, 0x00FCFCE7,
        0x00A59EF6, 0x007669FF, 0x007569FF, 0x007669FF, 0x007669FF, 0x007669FF, 0x007669FF, 0x007669FF, 0x007669FF,
        0x007669FF, 0x007569FF, 0x007569FF, 0x007669FF, 0x007669FF, 0x007568FF, 0x007568FF, 0x007568FF, 0x007568FF,
        0x007568FF, 0x007568FF, 0x007568FF, 0x009A92F8, 0x00FCFCE7, 0x008C84F9, 0x007568FF, 0x00867BFB, 0x00FCFCE7,
        0x00FCFCE7, 0x00FCFCE7, 0x00F8F8E8, 0x007D72FC, 0x00796DFE, 0x00EDECEA, 0x00FCFCE7, 0x00F6F6E8, 0x008F87F8,
        0x007367FF, 0x007568FF, 0x007568FF, 0x007568FF, 0x007568FF, 0x007568FF, 0x008A81FA, 0x00F0EFE9, 0x00FCFCE7,
        0x00F5F5E8, 0x008176FC, 0x007568FF, 0x007568FF, 0x007568FF, 0x007568FF, 0x007568FF, 0x007568FF, 0x007568FF,
        0x007568FF, 0x007568FF, 0x007568FF, 0x007568FF, 0x007568FF, 0x007467FF, 0x007467FF, 0x007467FF, 0x007467FF,
        0x007467FF, 0x007467FF, 0x007467FF, 0x00C0BBF1, 0x00FBFBE7, 0x007367FE, 0x007467FF, 0x00877DFB, 0x00FCFCE7,
        0x00FCFCE7, 0x00FCFCE7, 0x00CECBEE, 0x007266FF, 0x00ACA5F4, 0x00FCFCE7, 0x00FCFCE7, 0x00A19AF5, 0x007366FF,
        0x007467FF, 0x007467FF, 0x007367FF, 0x007467FF, 0x007467FF, 0x007467FF, 0x007467FF, 0x00968EF8, 0x00FCFCE7,
        0x00FCFCE7, 0x00BBB6F2, 0x007467FF, 0x007467FF, 0x007467FF, 0x007467FF, 0x007367FF, 0x007467FF, 0x007467FF,
        0x007467FF, 0x007467FF, 0x007467FF, 0x007467FF, 0x007467FF, 0x007366FF, 0x007366FF, 0x007366FF, 0x007366FF,
        0x007366FF, 0x007466FF, 0x007366FF, 0x00D7D4ED, 0x00F6F6E8, 0x006D60FF, 0x007366FF, 0x00968DF8, 0x00FCFCE7,
        0x00FCFCE7, 0x00FCFCE7, 0x00A9A3F4, 0x007366FF, 0x00DAD7ED, 0x00FCFCE7, 0x00E6E5EA, 0x009D95F6, 0x00CFCCEF,
        0x007367FF, 0x007366FF, 0x007366FF, 0x007366FF, 0x007466FF, 0x007366FF, 0x007366FF, 0x007366FF, 0x00D8D6ED,
        0x00FCFCE7, 0x00EAE9EA, 0x007164FF, 0x007366FF, 0x007366FF, 0x007366FF, 0x007366FF, 0x007366FF, 0x007366FF,
        0x007366FF, 0x007366FF, 0x007366FF, 0x007366FF, 0x007366FF, 0x007265FF, 0x007265FF, 0x007265FF, 0x007265FF,
        0x007265FF, 0x007265FF, 0x007265FF, 0x00D9D6EC, 0x00F8F8E8, 0x006D61FF, 0x007265FF, 0x009087F9, 0x00FCFCE7,
        0x00FCFCE7, 0x00FCFCE7, 0x009188F8, 0x007366FE, 0x00F3F3E8, 0x00FCFCE7, 0x00BCB8F1, 0x009F98F6, 0x00CCC9EE,
        0x007064FF, 0x007265FF, 0x007265FF, 0x007265FF, 0x007265FF, 0x007265FF, 0x007265FF, 0x007265FF, 0x00B0A9F4,
        0x00FCFCE7, 0x00FCFCE7, 0x00786DFD, 0x007265FF, 0x007265FF, 0x007265FF, 0x007265FF, 0x007265FF, 0x007265FF,
        0x007265FF, 0x007265FF, 0x007265FF, 0x007265FF, 0x007265FF, 0x007265FF, 0x007265FF, 0x007265FF, 0x007265FF,
        0x007265FF, 0x007265FF, 0x007264FF, 0x00D0CCEE, 0x00FCFCE7, 0x007D72FC, 0x007265FF, 0x00766AFE, 0x00F7F7E8,
        0x00FCFCE7, 0x00FCFCE7, 0x008C83F9, 0x007366FE, 0x00FAFAE7, 0x00FCFCE7, 0x00AEA8F3, 0x008378FC, 0x00857BFB,
        0x007265FF, 0x007265FF, 0x007265FF, 0x007264FF, 0x007265FF, 0x007264FF, 0x007265FF, 0x007265FF, 0x00A199F6,
        0x00FCFCE7, 0x00FCFCE7, 0x007F74FB, 0x007265FF, 0x007265FF, 0x007265FF, 0x007265FF, 0x007265FF, 0x007265FF,
        0x007265FF, 0x007265FF, 0x007265FF, 0x007265FF, 0x007265FF, 0x007164FF, 0x007164FF, 0x007164FF, 0x007164FF,
        0x007164FF, 0x007164FF, 0x007164FF, 0x00B4AFF3, 0x00FCFCE7, 0x00A29BF6, 0x007164FF, 0x007164FF, 0x00BDB8F1,
        0x00FCFCE7, 0x00FCFCE7, 0x008F86F8, 0x007265FE, 0x00F5F4E8, 0x00FCFCE7, 0x00BAB5F1, 0x00D7D4ED, 0x00EDECE9,
        0x007165FE, 0x007164FF, 0x007164FF, 0x007164FF, 0x007164FF, 0x007164FF, 0x007164FF, 0x007164FF, 0x00ACA5F4,
        0x00FCFCE7, 0x00FCFCE7, 0x00776CFD, 0x007164FF, 0x007164FF, 0x007164FF, 0x007164FF, 0x007164FF, 0x007164FF,
        0x007164FF, 0x007164FF, 0x007164FF, 0x007164FF, 0x007164FF, 0x007063FF, 0x007063FF, 0x007063FF, 0x007063FF,
        0x007063FF, 0x007063FF, 0x007063FF, 0x00897FFA, 0x00FCFCE7, 0x00DDDAEC, 0x007063FF, 0x007063FF, 0x007467FE,
        0x00D6D4ED, 0x00FCFCE7, 0x00A69FF5, 0x007062FF, 0x00DDDAEC, 0x00FCFCE7, 0x00E2E0EB, 0x00AEA8F3, 0x00FCFCE7,
        0x00ACA5F4, 0x007063FF, 0x007063FF, 0x007063FF, 0x007063FF, 0x007063FF, 0x007063FF, 0x007063FF, 0x00D2CEEE,
        0x00FCFCE7, 0x00EDEDE9, 0x006C5FFF, 0x007063FF, 0x007063FF, 0x007063FF, 0x007063FF, 0x007063FF, 0x007063FF,
        0x007063FF, 0x007063FF, 0x007063FF, 0x007063FF, 0x007063FF, 0x007062FF, 0x007062FF, 0x006F62FF, 0x007062FF,
        0x007062FF, 0x007062FF, 0x006F62FF, 0x006F62FF, 0x00DAD7EC, 0x00FCFCE7, 0x00A69FF5, 0x006F62FF, 0x007062FF,
        0x00786CFD, 0x00FAFAE7, 0x00CAC6EE, 0x006F62FF, 0x00AFA8F4, 0x00FCFCE7, 0x00FCFCE7, 0x009F97F6, 0x00E6E4EA,
        0x00FAFAE7, 0x00B7B1F2, 0x008074FC, 0x006F62FF, 0x006F62FF, 0x006F62FF, 0x007062FF, 0x008B80FA, 0x00FBFBE7,
        0x00FCFCE7, 0x00BBB7F1, 0x006F62FF, 0x007062FF, 0x006F62FF, 0x006F62FF, 0x006F62FF, 0x007062FF, 0x007062FF,
        0x006F62FF, 0x006F62FF, 0x006F62FF, 0x006F62FF, 0x007062FF, 0x006F61FF, 0x006F61FF, 0x006F61FF, 0x006F61FF,
        0x006F61FF, 0x006F61FF, 0x006F61FF, 0x006F61FF, 0x008B81FA, 0x00FBFBE7, 0x00F8F7E8, 0x00978EF7, 0x006F61FF,
        0x00867BFA, 0x00FBFBE7, 0x00F6F5E8, 0x00786CFD, 0x00766AFD, 0x00F1F1E9, 0x00FCFCE7, 0x00F2F1E8, 0x00978FF7,
        0x00E0DEEB, 0x00FCFCE7, 0x00FBFBE7, 0x00877DFA, 0x006F61FF, 0x006F61FF, 0x007F73FC, 0x00E9E7EA, 0x00FCFCE7,
        0x00F8F8E8, 0x007C72FB, 0x006F61FF, 0x006F62FF, 0x006F61FF, 0x006F61FF, 0x006F61FF, 0x006F61FF, 0x006F61FF,
        0x006F61FF, 0x006F61FF, 0x006F61FF, 0x006F61FF, 0x006F61FF, 0x006E61FF, 0x006E61FF, 0x006E61FF, 0x006E60FF,
        0x006E61FF, 0x006E61FF, 0x006E61FF, 0x006E61FF, 0x006E60FF, 0x00ADA7F3, 0x00FCFCE7, 0x00FBFBE7, 0x00DBD8EC,
        0x00F3F3E8, 0x00FCFCE7, 0x00FCFCE7, 0x00BAB4F1, 0x006E61FF, 0x009991F7, 0x00FBFBE7, 0x00FCFCE7, 0x00F6F6E8,
        0x00B1ACF3, 0x00A29CF5, 0x00ADA7F3, 0x006F62FE, 0x00776BFD, 0x00A8A1F5, 0x00F1F1E9, 0x00FCFCE7, 0x00FCFCE7,
        0x00E0DEEB, 0x007569FD, 0x008176FB, 0x006E61FF, 0x006E60FF, 0x006E61FF, 0x006E61FF, 0x006E60FF, 0x006E61FF,
        0x006E61FF, 0x006E61FF, 0x006E60FF, 0x006E61FF, 0x006E61FF, 0x006D60FF, 0x006D60FF, 0x006D60FF, 0x006E60FF,
        0x006E60FF, 0x006D60FF, 0x006D60FF, 0x006E60FF, 0x006E60FF, 0x006D60FF, 0x00B2ACF2, 0x00FBFBE7, 0x00FCFCE7,
        0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00F9F9E7, 0x008D83F9, 0x006D5FFF, 0x00A59DF5, 0x00FAF9E7, 0x00FCFCE7,
        0x00FCFCE7, 0x00FAFAE7, 0x00EDECE9, 0x00ECEBEA, 0x00F9F9E7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7,
        0x00FCFCE7, 0x00F6F5E8, 0x00FCFCE7, 0x00B8B3F2, 0x006E60FF, 0x006D60FF, 0x006E60FF, 0x006D60FF, 0x006E60FF,
        0x006E60FF, 0x006E60FF, 0x006E60FF, 0x006E60FF, 0x006D60FF, 0x006D5FFF, 0x006D5FFF, 0x006D5FFF, 0x006D5FFF,
        0x006D5FFF, 0x006D5FFF, 0x006D5FFF, 0x006D5FFF, 0x006D5FFF, 0x006D5FFF, 0x006C5FFF, 0x00958CF7, 0x00EBEAE9,
        0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00F0EFE9, 0x008479FB, 0x006D5FFF, 0x008A80F9, 0x00DEDCEC,
        0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00E5E4EA, 0x00948CF7, 0x00D1CEED,
        0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00B8B2F1, 0x006C5FFF, 0x006D5FFF, 0x006D5FFF, 0x006D5FFF,
        0x006D5FFF, 0x006D5FFF, 0x006D5FFF, 0x006D5FFF, 0x006D5FFF, 0x006B5EFF, 0x006C5EFF, 0x006B5EFF, 0x006C5EFF,
        0x006B5EFF, 0x006C5EFF, 0x006C5EFF, 0x006C5EFF, 0x006C5EFF, 0x006C5EFF, 0x006C5EFF, 0x006C5EFF, 0x006D61FE,
        0x009F98F5, 0x00CFCDED, 0x00F2F2E8, 0x00FCFCE7, 0x00FCFCE7, 0x00EFEFE9, 0x00887EFA, 0x006B5EFF, 0x006A5DFF,
        0x00867CF9, 0x00ACA6F3, 0x00BFBBF0, 0x00C0BCF0, 0x00AEA9F3, 0x008A81F8, 0x00685CFF, 0x006B5EFF, 0x00BEB8F1,
        0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00B8B2F2, 0x006C5FFF, 0x006C5EFF, 0x006C5EFF,
        0x006C5EFF, 0x006C5EFF, 0x006C5EFF, 0x006C5EFF, 0x006B5EFF, 0x006B5EFF, 0x006B5EFF, 0x006B5EFF, 0x006B5EFF,
        0x006B5EFF, 0x006B5EFF, 0x006B5DFF, 0x006B5EFF, 0x006B5EFF, 0x006B5DFF, 0x006B5EFF, 0x006B5EFF, 0x006B5EFF,
        0x006B5EFF, 0x006A5DFF, 0x00695CFF, 0x007266FD, 0x00756AFD, 0x006A5EFE, 0x00695CFF, 0x006B5EFF, 0x006B5EFF,
        0x006B5EFF, 0x006B5EFF, 0x006B5EFF, 0x006B5DFF, 0x006B5DFF, 0x006B5EFF, 0x006B5EFF, 0x006B5EFF, 0x00A7A0F4,
        0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00AFA8F3, 0x006B5EFF, 0x006B5EFF,
        0x006B5EFF, 0x006B5EFF, 0x006B5EFF, 0x006B5DFF, 0x006B5EFF, 0x006B5DFF, 0x006B5DFF, 0x006B5DFF, 0x006B5DFF,
        0x006B5DFF, 0x006B5DFF, 0x006A5DFF, 0x006B5DFF, 0x006B5DFF, 0x006B5DFF, 0x006B5DFF, 0x006B5DFF, 0x006B5DFF,
        0x006B5DFF, 0x006B5DFF, 0x006B5DFF, 0x006A5DFF, 0x006B5DFF, 0x006B5DFF, 0x006B5DFF, 0x006A5DFF, 0x006B5DFF,
        0x006A5DFF, 0x006B5DFF, 0x006B5DFF, 0x006B5DFF, 0x006B5DFF, 0x006B5DFF, 0x006B5DFF, 0x006B5DFF, 0x006A5DFF,
        0x00B4AEF2, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00D1CDED, 0x006A5DFF, 0x006B5DFF,
        0x006B5DFF, 0x006B5DFF, 0x006B5DFF, 0x006B5DFF, 0x006B5DFF, 0x006A5CFF, 0x006A5CFF, 0x006A5CFF, 0x006A5CFF,
        0x006A5CFF, 0x006A5CFF, 0x006A5CFF, 0x006A5CFF, 0x006A5CFF, 0x006A5CFF, 0x006A5CFF, 0x006A5CFF, 0x006A5CFF,
        0x006A5CFF, 0x006A5CFF, 0x006A5CFF, 0x006A5CFF, 0x006A5CFF, 0x006A5CFF, 0x006A5CFF, 0x006A5CFF, 0x006A5CFF,
        0x006A5CFF, 0x006A5CFF, 0x006A5CFF, 0x006A5CFF, 0x006A5CFF, 0x006A5CFF, 0x006A5CFF, 0x006A5CFF, 0x006A5CFF,
        0x00695CFF, 0x00B3ADF2, 0x00FCFCE7, 0x00FCFCE7, 0x00FCFCE7, 0x00F2F1E8, 0x008178F9, 0x006A5CFF, 0x006A5CFF,
        0x006A5CFF, 0x006A5CFF, 0x006A5CFF, 0x006A5CFF, 0x006A5CFF, 0x04695CFF, 0x00695CFF, 0x00695CFF, 0x00695CFF,
        0x00695CFF, 0x00695CFF, 0x00695CFF, 0x00695CFF, 0x00695CFF, 0x006A5CFF, 0x00695CFF, 0x00695CFF, 0x00695CFF,
        0x00695CFF, 0x00695CFF, 0x00695CFF, 0x00695CFF, 0x00695CFF, 0x00695CFF, 0x00695CFF, 0x006A5CFF, 0x00695CFF,
        0x00695CFF, 0x00695CFF, 0x00695CFF, 0x00695CFF, 0x00695CFF, 0x00695CFF, 0x00695CFF, 0x006A5CFF, 0x00695CFF,
        0x00695CFF, 0x00695BFF, 0x00B3ADF2, 0x00FCFCE7, 0x00F2F1E8, 0x008177FA, 0x00695BFF, 0x00695CFF, 0x00695CFF,
        0x006A5CFF, 0x00695CFF, 0x00695CFF, 0x00695CFF, 0x04695CFF, 0x1E6155EE, 0x00695BFF, 0x00695BFF, 0x00695BFF,
        0x00695BFF, 0x00695BFF, 0x00695BFF, 0x00695BFF, 0x00695BFF, 0x00695BFF, 0x00695BFF, 0x00695BFF, 0x00695BFF,
        0x00695BFF, 0x00695BFF, 0x00695BFF, 0x00695BFF, 0x00695BFF, 0x00695BFF, 0x00695BFF, 0x00695BFF, 0x00695BFF,
        0x00695BFF, 0x00695BFF, 0x00695BFF, 0x00695BFF, 0x00695BFF, 0x00695BFF, 0x00695BFF, 0x00695BFF, 0x00695BFF,
        0x00695BFF, 0x00695BFF, 0x00695BFF, 0x00857BF9, 0x007469FC, 0x00695BFF, 0x00695BFF, 0x00695BFF, 0x00695BFF,
        0x00695BFF, 0x00695BFF, 0x00695BFF, 0x00695BFF, 0x1E6255EE, 0x5E493FB5, 0x00685AFF, 0x00695AFF, 0x00685BFF,
        0x00685BFF, 0x00685BFF, 0x00685AFF, 0x00685AFF, 0x00685BFF, 0x00695BFF, 0x00695AFF, 0x00685AFF, 0x00685AFF,
        0x00685AFF, 0x00685BFF, 0x00685AFF, 0x00685AFF, 0x00685BFF, 0x00685AFF, 0x00685BFF, 0x00685AFF, 0x00685AFF,
        0x00685BFF, 0x00685AFF, 0x00685AFF, 0x00685AFF, 0x00685AFF, 0x00695AFF, 0x00685AFF, 0x00685AFF, 0x00685AFF,
        0x00695AFF, 0x00695AFF, 0x00685BFF, 0x00685AFF, 0x00685AFF, 0x00685AFF, 0x00685AFF, 0x00685AFF, 0x00695BFF,
        0x00685AFF, 0x00685AFF, 0x00685AFF, 0x00685AFF, 0x5E493FB5, 0xC21E1A4C, 0x01685AFE, 0x00685AFF, 0x00685AFF,
        0x00685AFF, 0x00685AFF, 0x00685AFF, 0x00685AFF, 0x00685AFF, 0x00685AFF, 0x00685AFF, 0x00685AFF, 0x00685AFF,
        0x00685AFF, 0x00685AFF, 0x00685AFF, 0x00685AFF, 0x00685AFF, 0x00685AFF, 0x00685AFF, 0x00685AFF, 0x00685AFF,
        0x00685AFF, 0x00685AFF, 0x00685AFF, 0x00685AFF, 0x00685AFF, 0x00685AFF, 0x00685AFF, 0x00685AFF, 0x00685AFF,
        0x00685AFF, 0x00685AFF, 0x00685AFF, 0x00685AFF, 0x00685AFF, 0x00685AFF, 0x00685AFF, 0x00685AFF, 0x00685AFF,
        0x00685AFF, 0x00685AFF, 0x00685AFF, 0x01685AFE, 0xC21E1A4C, 0xFF000000, 0x574D45B3, 0x006859FF, 0x00685AFF,
        0x00685AFF, 0x006859FF, 0x006859FF, 0x00685AFF, 0x00685AFF, 0x00685AFF, 0x006859FF, 0x006859FF, 0x006859FF,
        0x00685AFF, 0x00685AFF, 0x00685AFF, 0x00685AFF, 0x006859FF, 0x006859FF, 0x00685AFF, 0x006859FF, 0x00685AFF,
        0x00685AFF, 0x00685AFF, 0x006859FF, 0x006859FF, 0x006859FF, 0x006859FF, 0x00685AFF, 0x00685AFF, 0x006859FF,
        0x006859FF, 0x00685AFF, 0x00685AFF, 0x006859FF, 0x00685AFF, 0x006859FF, 0x00685AFF, 0x006859FF, 0x006859FF,
        0x006859FF, 0x006859FF, 0x006859FF, 0x574E45B3, 0xFF000000, 0xFF000000, 0xEF090817, 0x33564AD4, 0x006759FF,
        0x006759FF, 0x006759FF, 0x006759FF, 0x006759FF, 0x006759FF, 0x006759FF, 0x006759FF, 0x006759FF, 0x006759FF,
        0x006759FF, 0x006759FF, 0x006759FF, 0x006759FF, 0x006759FF, 0x006759FF, 0x006759FF, 0x006759FF, 0x006759FF,
        0x006759FF, 0x006759FF, 0x006759FF, 0x006759FF, 0x006759FF, 0x006759FF, 0x006759FF, 0x006759FF, 0x006759FF,
        0x006759FF, 0x006759FF, 0x006759FF, 0x006759FF, 0x006759FF, 0x006759FF, 0x006759FF, 0x006759FF, 0x006759FF,
        0x006759FF, 0x006759FF, 0x33564AD4, 0xEF090817, 0xFF000000, 0xFF000000, 0xFF000000, 0xEF090817, 0x574E44B3,
        0x016759FE, 0x006759FF, 0x006759FF, 0x006759FF, 0x006759FF, 0x006759FF, 0x006759FF, 0x006759FF, 0x006759FF,
        0x006759FF, 0x006759FF, 0x006759FF, 0x006759FF, 0x006759FF, 0x006759FF, 0x006759FF, 0x006759FF, 0x006759FF,
        0x006659FF, 0x006759FF, 0x006759FF, 0x006759FF, 0x006759FF, 0x006758FF, 0x006759FF, 0x006759FF, 0x006759FF,
        0x006759FF, 0x006759FF, 0x006759FF, 0x006658FF, 0x006759FF, 0x006759FF, 0x006759FF, 0x006759FF, 0x006759FF,
        0x016759FE, 0x574E44B3, 0xEF090817, 0xFF000000, 0xFF000000, 0xFF000000, 0xFF000000, 0xFF000000, 0xFF000000,
        0xC21E1A4C, 0x5E473DB4, 0x1E5F52EE, 0x046658FF, 0x006658FF, 0x006658FF, 0x006658FF, 0x006658FF, 0x006658FF,
        0x006658FF, 0x006658FF, 0x006658FF, 0x006658FF, 0x006658FF, 0x006658FF, 0x006658FF, 0x006658FF, 0x006658FF,
        0x006658FF, 0x006658FF, 0x006658FF, 0x006658FF, 0x006658FF, 0x006658FF, 0x006658FF, 0x006658FF, 0x006658FF,
        0x006658FF, 0x006658FF, 0x006658FF, 0x006658FF, 0x006658FF, 0x006658FF, 0x046658FF, 0x1E5F52EE, 0x5E483DB4,
        0xC21E194C, 0xFF000000, 0xFF000000, 0xFF000000, 0xFF000000 };

static GUI_CONST_STORAGE GUI_BITMAP bmSPO2ValueIcon = { 45,     // xSize
        45,     // ySize
        180,     // BytesPerLine
        32,     // BitsPerPixel
        (unsigned char *) _acSPO2ValueIcon,     // Pointer to picture data
        NULL,     // Pointer to palette
        GUI_DRAW_BMP8888 };

/*************************** End of file ****************************/
