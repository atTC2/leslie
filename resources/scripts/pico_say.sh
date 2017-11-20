#!/bin/bash

trap 'exit' ERR

# --- Constant definitions. ---

# x is empty.
x=''
# The name of this script.
scriptName='pico_say.sh'
# Float regex.
float_regex='^[0-9]+([.][0-9]+)?$'

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
if ! [[ "${1}" =~ ${float_regex} ]]; then
    error_exit "Tempo must be a positive decimal." 2
fi

# --- --- ---


# --- Main script. ---

# Do our dirty work in a temporary directory.
tmpDir="$(mktemp -d)"

# Clean up on exit, always.
function finish {
  rm -rf "${tmpDir}"
}
trap finish EXIT

# TTS.
wav0="${tmpDir}/out0.wav"
pico2wave -l=en-GB -w="${wav0}" "${2}"

# Speed up Leslie's voice.
wav1="${tmpDir}/out1.wav"
sox "${wav0}" "${wav1}" vol 0.8 tempo "${1}"

# Play the speech.
aplay -q "${wav1}"

# --- --- ---
