#pragma once
// Базовая версия программы
#define BASE_PROGRAM_VERSION "1.0.9"
// Дополнительная версия программы (значение будет подставленно вместо 8 утилитой SubWCRev.exe)
#define ADDITIONAL_PROGRAM_VERSION "9"
// Тип выпускаемого релиза (pre-alpha, alpha, beta, rc, rtm...)
#define RELEASE_TYPE "beta"
// Желаемый формат строки версии
#define PROGRAM_VERSION BASE_PROGRAM_VERSION##"."##ADDITIONAL_PROGRAM_VERSION##" "##RELEASE_TYPE