#include "AppData.h"

AppData& GetApplicationData() {
    static AppData data;
    return data;
}