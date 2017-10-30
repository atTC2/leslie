#!/bin/bash

set -e

# --- Constant definitions. ---

# x is empty.
x=''
# The name of this script.
scriptName='install_one.sh'

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

if [ -z "${1+x}" ]; then
    error_exit "Usage: ./${scriptName} script_name" 2
fi
if [[ "${1}" != *'.sh' ]]; then
    error_exit "Can only install '.sh' files." 2
fi
if [[ "${1}" == 'install_'*'.sh' ]]; then
    error_exit 'Cannot install the install util scripts.' 2
fi

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


# --- Install the script. ---

f="${scriptLocation}/${1}"

echo 'vvv Heading vvv'
echo 'Script:'
echo "${f}"
echo '^^^ Heading ^^^'

# Installed check.
if [ -e "${f}.installed" ]; then
    echo 'Already installed.'
    echo '### ### ###'
    echo
    exit 0
fi

# Existence check.
if [[ ! -e "${f}" ]]; then
    error_exit 'Script not found.' 2
fi

# Executable check.
if [[ ! -x "${f}" ]]; then
    error_exit 'Script is not executable.' 2
fi

# Optional check.
if [[ "${1}" == *'_opt_'* ]]; then
    # Ask if they want to install the optional script.
    while true; do
        read -n 1 -p 'Do you want to install this optional script (y/n)? ' choice
        echo
        case "$choice" in
            y|Y )
                break
                ;;
            n|N )
                echo 'User said they did not want to install the optional script.'
                echo '### ### ###'
                echo
                exit 0
                ;;
            * )
                echo 'Invalid option.'
                ;;
        esac
    done
fi

# Run the script.
echo 'vvv Script output vvv'
if [[ "${1}" == *'_sudo_'* ]] || [[ "${1}" == *'_opt_sudo_'* ]]; then
    sudo "${f}"
    exitCode=$?
else
    bash "${f}"
    exitCode=$?
fi
echo '^^^ Script output ^^^'

# Check the exit status.
if [ "${exitCode}" -ne 0 ]; then
    error_exit "Something went wrong with the scripts execution... (Exit code: ${exitCode})" 3
fi

# Ask user to check the output.
while true; do
    read -n 1 -p 'Please check the output. Did the script install correctly (y/n)? ' choice
    echo
    case "$choice" in
        y|Y )
            break
            ;;
        n|N )
            error_exit 'User said that the script did not install correctly.' 4
            ;;
        * )
            echo 'Invalid option.'
            ;;
    esac
done

# Everything went well, add a '.installed' file.
touch "${f}.installed" || error_exit "Failed to create file '${f}.installed' please create it manually."
echo 'Installed!'
echo '^^^ ^^^ ^^^'
echo

# --- --- ---
