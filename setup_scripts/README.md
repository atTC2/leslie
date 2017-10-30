# Setup scripts

These scripts are used to install the required components for this project.

## Running

To install all the scripts simply run `./install_all.sh`.  
To install a script individually run `./install_one.sh SCRIPT_NAME`, where `SCRIPT_NAME` is the name of the script to install.

## '.installed' files

These files are created when an install script has been successfully run, to indicate that it does not need to be run again.

## Script name format

Format: `ddd_[opt_][sudo_]name.sh`, where; `ddd` are numbers; the sections in square brackets are optional; and `name` is a unique name.

#### Ordering

To ensure the correct execution order of the install scripts, they must start with a 3 digit number. `0xx` and `00x` are allowed.

#### Optional install scripts

To indicate that an install script is optional, include `_opt` after the ordering numbers. E.g. `001_opt_name.sh`.

#### Running install scripts as root

To indicate that an install script should be run as root, include `_sudo` after the ordering numbers and after `_opt`, if applicable. E.g. `099_sudo_name.sh` and `040_opt_sudo_name.sh`.
