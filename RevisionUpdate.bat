:: ������ "�����"
set LAPKA="
:: �������� ���������� ����, ��� ��������� ������ ������
set UPATH=%~dp0
:: �������� ������� backslash-�� �� forwardslash-� (��� ����� ��� ���������� ������ ��������� SubWCRev.exe)
set UPATH=%UPATH:\=/%
:: ��������� ��� ���������� �����
set TEMPLATE=Revision.templ
:: ��������� ��� ��������������� �����
set FILE=Revision.h

:: ��������� ������� � ������������ ����
SubWCRev.exe %LAPKA%%UPATH%%LAPKA% %LAPKA%%UPATH%%TEMPLATE%%LAPKA% %LAPKA%%UPATH%%FILE%%LAPKA%


C:\Keil_v5\UV4\UV4.exe -b Projects\MDK-ARM\BHD2.uvprojx -o "log.txt"      
