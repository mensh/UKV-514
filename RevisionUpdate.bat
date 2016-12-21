:: Символ "лапка"
set LAPKA="
:: Получаем абсолютный путь, где находится данный скрипт
set UPATH=%~dp0
:: Заменяем символы backslash-ов на forwardslash-и (это нужно для корректной работы программы SubWCRev.exe)
set UPATH=%UPATH:\=/%
:: Указываем имя шаблонного файла
set TEMPLATE=Revision.templ
:: Указываем имя результирующего файла
set FILE=Revision.h

:: Обновляем ревизию в заголовочный файл
SubWCRev.exe %LAPKA%%UPATH%%LAPKA% %LAPKA%%UPATH%%TEMPLATE%%LAPKA% %LAPKA%%UPATH%%FILE%%LAPKA%


C:\Keil_v5\UV4\UV4.exe -b Projects\MDK-ARM\BHD2.uvprojx -o "log.txt"      
