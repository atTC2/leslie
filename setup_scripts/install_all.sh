#!/bin/bash

set -e

# --- Constant definitions. ---

# x is empty.
x=''
# The name of this script.
scriptName='install_all.sh'

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


# --- Determine the location of this script. ---

if [ ! -z "${0+x}" ]\
&& [ -e "${0}" ]\
&& [[ "${0}" == *"${scriptName}" ]]\
&& [ $(basename "${0}") = "${scriptName}" ]; then
    scriptLocation="$(dirname "${0}")"
elif [ -e "$(pwd)/${scriptName}" ]; then
    scriptLocation="$(pwd)"
elif [ -e "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/${scriptName}" ]; then
    scriptLocation="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
else
    error_exit 'Can not determine the location of this script...\nPlease run this script from the same folder as the script.' 1
fi

# --- --- ---


# --- Run the scripts. ---

for f in "${scriptLocation}/"*'.sh'; do
    # Ignore the install scripts.
    if [[ "${f}" == *'/install_'*'.sh' ]]; then
        continue
    fi
    bash "${scriptLocation}/install_one.sh" "$(basename "${f}")" || exit $?
done

# --- --- ---
