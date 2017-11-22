#!/bin/bash

trap 'exit' ERR

# --- Constant definitions. ---

# x is empty.
x=''
# The name of this script.
scriptName='pico_say.sh'
# Float regex.
floatRegex='^[0-9]+([.][0-9]+)?$'
# Directory to lock on and work in.
workDir="/tmp/pico_say"
# PID file for fun.
pidFile="${workDir}/PID"

# --- --- ---


# --- Function definitions. ---

function error_exit
{
#   ----------------------------------------------------------------
#   Print error message and exit.
#       Accepts 2 argument:
#           string - optional - error message
#           integer - optional - the exit code
#   ----------------------------------------------------------------
    echo "${scriptName}: ${1:-"Unknown Error"}" 1>&2
    exit ${2:-"1"}
}

# --- --- ---


# --- Check arguments. ---

if [ -z "${1+x}" ] || [ -z "${2+x}" ]; then
    error_exit "Usage: ./${scriptName} tempo text" 2
fi
if ! [[ "${1}" =~ ${floatRegex} ]]; then
    error_exit "Tempo must be a positive decimal." 2
fi

# --- --- ---


# --- Mutex the script. ---

# Acquire a lock, to ensure two speech processes aren't running at once.
until mkdir "${workDir}" &>/dev/null
do
    sleep 0.5
done

function removeLock {
    rm -rf "${workDir}"
}
trap removeLock EXIT

echo "$$" >"${pidFile}"

# --- --- ---


# --- Main script. ---

# TTS.
wav0="${workDir}/out0.wav"
pico2wave -l=en-GB -w="${wav0}" "${2}"

# Speed up Leslie's voice.
wav1="${workDir}/out1.wav"
sox "${wav0}" "${wav1}" vol 0.8 tempo "${1}"

# Play the speech.
aplay -q "${wav1}"

# --- --- ---
