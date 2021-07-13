#! /bin/bash

source ../../tools/settings.sh

# Launch export_hw
echo Launching export_hw
rdk export_hw

re="^tmp_hls_prj_"
errors=0;

# Log file eval
cd build.hw
for D in *; do
	if [ -d "${D}" ]; then
		if [[ ${D} =~ $re ]]; then
			if grep -q ERROR "${D}/vivado_hls.log"; then
				echo Found error in ${D}/vivado_hls.log
				errors=1
			fi
		fi
	fi
done

cd ..
# Condition checking
if [ ${errors} -eq "0" ]; then
	echo No errors found, launching build_hw ...
	rdk build_hw
else
	echo Review the log files, there are errors!
fi
