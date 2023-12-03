# Script options:
diff=true
update_link="https://go.asacco.dev/cst"
script_name="CSTest"
version="2.0.0rc11"

# Text formatting
txg=$(tput setaf 2)
txr=$(tput setaf 1)
txy=$(tput setaf 3)
txb=$(tput setaf 4)
txh=$(tput bold)
txc=$(tput sgr0)
filename=$(basename "$0")
tc_name_regex="([a-zA-Z0-9+/=]+)"


# -v or --version
if [ "$1" == "-v" ] || [ "$1" == "--version" ]
then
    # Print version information
    echo "$txb$txh$script_name$txc$txb v$version$txc"
    exit 0
fi

# -h or --help (or -hh)
if [ "$1" == "-h" ] || [ "$1" == "--help" ] || [ "$1" == "-hh" ]
then
    # Print help message
    echo "$txy$txh$script_name$txc$txy v$version$txc"
    echo "Usage: ./$filename [options|id(s)]\n"
    echo "${txb}With no arguments,${txc} this script runs all tests in the current folder. Failed tests"
    echo "will print the ${txh}diff${txc} between the ${txg}expected output$txc and the ${txr}actual output${txc}.\n"
    echo "${txb}If only non-option arguments are provided,${txc} they are treated as test ids to run."
    echo "For example, $txy./$filename 1 3$txc will run tests 1 and 3.\n"
    echo "${txh}Options:${txc}"
    echo "  $txh-u$txc                  Update this script to latest version\n"
    echo "  $txh-s$txc=<name> [id(s)]   Save a test pack using any provided test numbers, else use"
    echo "                      all tests in the current folder\n"
    echo "  $txh-a$txc [file]           Add a test pack to the current folder. $txg(Non-destructive)$txc"
    echo "  $txh-l$txc [file]           Load a test pack. $txr${txh}THIS OVERWRITES ALL TESTS IN THE CURRENT"
    echo "                      FOLDER$txc$txr (so use $txc$txh-s$txc$txr to make a backup first!${txr})$txc\n"
    echo "  $txh-h$txc                  Display this help message. Use $txh-hh$txc for extra tools\n"
    echo "  $txh-v$txc                  Print version information\n"
    if [ "$1" == "-hh" ]
    then
        # Print extra tools
        echo "\n${txh}Extra tools:$txc"
        echo "  $txh-e$txc [id]             Export the contents of in[id].txt and out[id].txt as a base64 string\n"
        echo "  $txh-i$txc [base64]         Import the contents of in.txt and out.txt from a base64 string\n"
        echo "  $txh-r$txc [id(s)]          Remove the test files in[id].txt and out[id].txt $txr(Destructive)$txc\n"
    fi
    exit 0
fi

# -e or --export
if [ "$1" == "-e" ] || [ "$1" == "--export" ]
then
    # Get the contents of in$2.txt and out$2.txt
    in_file="in$2.txt"
    out_file="out$2.txt"
    base64_string=$(echo $2:$(cat "$in_file" | base64):$(cat "$out_file" | base64))

    # Use the base64 string as needed
    echo "$base64_string"
    exit 0
fi

# -i or --import
if [ "$1" == "-i" ]
then
    # Get test name from the argument
    import_string="$2"
    i=$(echo "$import_string" | sed -n -E "s/^$tc_name_regex:([a-zA-Z0-9+/=]+):([a-zA-Z0-9+/=]+)$/\1/p")
    init_i=$i

    while [ -f "./in$i.txt" ]
    do
        i=$((i+1))
    done

    # Get the contents of in.txt and out.txt from a base64 string
    in_base64=$(echo "$import_string" | sed -n -E "s/^$tc_name_regex:([a-zA-Z0-9+/=]+):([a-zA-Z0-9+/=]+)$/\2/p")
    out_base64=$(echo "$import_string" | sed -n -E "s/^$tc_name_regex:([a-zA-Z0-9+/=]+):([a-zA-Z0-9+/=]+)$/\3/p")
    echo "$in_base64" | base64 -d > "in$i.txt"
    echo "$out_base64" | base64 -d > "out$i.txt"

    if [ $i != $init_i ]
    then
        echo "Imported test with ID $i (ID $init_i was already taken)"
    else
        echo "Imported test $i"
    fi

    exit 0
fi

# -l or --load, -a or --add
if [ "$1" == "-l" ] || [ "$1" == "-a" ]
then
    if [ $# -eq 1 ]
    then
        echo "Usage: ./$filename -l [file]"
        exit 1
    fi
    
    to_remove=()
    # remove all tests from current folder
    for file in in*.txt
    do
        to_remove+=("$file")
    done
    for file in out*.txt
    do
        to_remove+=("$file")
    done

    # for each line in file whose path is $2, import as a test like in -i
    printed_import=false
    while IFS= read -r line
    do
        import_string="$line"

        # Check if the line is a test pack header
        if [[ "$import_string" =~ ^c/([a-zA-Z0-9.+/=]+)/([a-zA-Z0-9+/=]+)$ ]]
        then
            cstp_version=$(echo "$import_string" | sed -n -E 's/^c\/([a-zA-Z0-9.+/=]+)\/([a-zA-Z0-9+/=]+)$/\1/p')
            echo "${txy}Test pack created with $script_name version $cstp_version${txc}"
            cstp_parent_folder=$(echo "$import_string" | sed -n -E 's/^c\/([a-zA-Z0-9.+/=]+)\/([a-zA-Z0-9+/=]+)$/\2/p')
            assignment=$(basename "$PWD" | sed -n -E 's/^([a-zA-Z0-9+/=]+)-([a-zA-Z0-9+/=]+)$/\1/p')

            # Check if the test pack was created for the current assignment
            if [ "$cstp_parent_folder" != "$assignment" ]
            then
                # If not, show a warning and prompt the user before continuing
                echo "${txr}Warning: This test pack may have been created for a different assignment!${txc}"
                echo "         It was created in a folder named ${txh}$cstp_parent_folder${txc}"

                printf "${txb}Are you sure you want to load this file?${txc} (${txg}y${txc}/${txr}n${txc}): "
                read -r response < /dev/tty
                if [ "$response" != "y" ]
                then
                    echo "Aborting"
                    exit 0
                fi
            fi
        else # Not a header, so import the test
            # Is this the first test being imported?
            if [ $printed_import == false ]
            then
                # If so, remove all tests from current folder (only if loading)
                if [ "$1" == "-l" ]
                then
                    # remove all tests from current folder
                    for file in ${to_remove[@]}
                    do
                        rm "$file"
                    done
                fi

                printf "Imported test"
                printed_import=true
            fi

            i=$(echo "$import_string" | sed -n -E "s/^$tc_name_regex:([a-zA-Z0-9+/=]+):([a-zA-Z0-9+/=]+)$/\1/p")

            init_i=$i
            if [ "$1" == "-a" ]
            then
                while [ -f "./in$i.txt" ]
                do
                    i=$((i+1))
                done
            fi

            # Get the contents of in.txt and out.txt from the current line
            in_base64=$(echo "$import_string" | sed -n -E "s/^$tc_name_regex:([a-zA-Z0-9+/=]+):([a-zA-Z0-9+/=]+)$/\2/p")
            out_base64=$(echo "$import_string" | sed -n -E "s/^$tc_name_regex:([a-zA-Z0-9+/=]+):([a-zA-Z0-9+/=]+)$/\3/p")
            echo "$in_base64" | base64 -d > "in$i.txt"
            echo "$out_base64" | base64 -d > "out$i.txt"
            
            if [ $i != $init_i ]
            then
                printf " $init_i(as $i)"
            else
                printf " $i"
            fi
            # printf " $i"
        fi
    done < "$2"
    echo
    exit 0
fi

if [ "$1" == "-r" ]
then
    if [ $# -eq 1 ]
    then
        echo "Usage: ./$filename -r [test ids]"
        exit 1
    fi

    confirm_all=false
    # loop through arguments
    for arg in "$@"
    do
        # skip first argument
        if [ "$arg" == "$1" ]
        then
            continue
        fi
        
        if [ $confirm_all == false ]
        then
            # show confirmation prompt
            printf "Are you sure you want to remove test $arg? (y/a/n): "
            read -r response
            if [ "$response" == "a" ]
            then
                confirm_all=true
            else
                if [ "$response" != "y" ]
                then
                    echo "Aborting"
                    exit 0
                fi
            fi
        fi

        # remove test files matching $arg
        if [ ! -f "./in$arg.txt" ]
        then
            if [ ! -f "./out$arg.txt" ]
            then
                echo "Test $arg does not exist"
            fi
        else
            rm "in$arg.txt"
            rm "out$arg.txt"
            echo "Removed Test $arg"
        fi
    done
    exit 0
fi

if [ "$1" == "-u" ]
then
    # check for updated version on server
    wget --no-cache -q $update_link -O $filename.new
    if [ $? -eq 0 ]; then
        if ! diff -q $filename $filename.new >/dev/null ; then
            # Replace this file with the new version
            echo "New version available. Updating..."
            rm $filename
            mv $filename.new $filename
            chmod +x "$filename"
            echo "${txg}Updated successfully.${txc}"
            exit 0
        fi
        rm $filename.new
        echo "Already up to date."
        exit 0
    fi
    rm $filename.new
    echo "Error checking for updates."
    exit 1
fi

# check if any there are any arguments that do not start with a dash
custom_files=false
for arg in "$@"
do
    if [ "${arg:0:1}" != "-" ]
    then
        custom_files=true
        break
    fi
done

if [ $custom_files == true ]
then
    fail=false
    matching_files=()
    for arg in "$@"
    do
        if [ "${arg:0:1}" != "-" ]
        then
            if [[ "$arg" == *\* ]]
            then
                for file in in$arg.txt
                do
                    t=$(echo "$file" | sed -n -E "s/^in$tc_name_regex\.txt$/\1/p")
                    if [ -f "./out$t.txt" ]
                    then
                        matching_files+=("$t")
                    else
                        fail=true
                        echo "No matching output file for $arg"
                    fi
                done
            else
                if [ -f "./out$arg.txt" ] && [ -f "./in$arg.txt" ]
                then
                    matching_files+=("$arg")
                else
                    fail=true
                    echo "No matching output file for $arg"
                fi
            fi
        fi
    done
    if [ $fail == true ]
    then
        exit 1
    fi
else
    matching_files=()
    for file in in*.txt
    do
        # Extract the number i from the file name
        i=$(echo "$file" | sed -n -E "s/^in$tc_name_regex\.txt$/\1/p")
        out_file="out$i.txt"

        # Check if there exists a corresponding out_file
        if [ -f "./$out_file" ]
        then
            matching_files+=("$i")
        else
            echo "Output file $out_file does not exist"
        fi
    done
fi

# Sort matching_files by putting all integer values first, sorted ascending, then all non-integer values
matching_files=($(printf '%s\n' "${matching_files[@]}" | sort -n -k1,1 | sort -V))

for i in "${matching_files[@]}"
do
    # Add newline to end of file if it doesn't exist
    if [ "$(tail -c 1 out$i.txt)" != "" ]; then
        echo >> out$i.txt
    fi
done

if [[ "$1" == -s=* ]]
then
    # get name from arg
    name=$(echo "$1" | sed -n -E 's/^-s=(.*)$/\1/p')

    if [ -f "$name.cstp" ]
    then
        printf "$txh${txr}Warning:$txc${txr} $name.cstp already exists!${txc}\n"
        printf "$txh${txy}Are you sure you want to overwrite it?${txc} (${txg}y${txc}/${txr}n${txc}): "
        read -r response
        if [ "$response" != "y" ]
        then
            echo "Aborting"
            exit 0
        fi
        rm "$name.cstp"
    fi

    printf "$txh${txb}Packing$txc$txb test$txc"
    assignment=$(basename "$PWD" | sed -n -E 's/^([a-zA-Z0-9+/=]+)-([a-zA-Z0-9+/=]+)$/\1/p')
    if [ "$assignment" == "" ]
    then
        assignment=$(basename "$PWD")
        printf "$assignment"
    fi
    echo "c/$version/$assignment" >> $name$ei.cstp

    for i in "${matching_files[@]}"
    do
        printf " $i"

        # Get the contents of in$2.txt and out$2.txt
        in_file="in$i.txt"
        out_file="out$i.txt"
        base64_string=$(echo $i:$(cat "$in_file" | base64):$(cat "$out_file" | base64))

        # Use the base64 string as needed
        echo "$base64_string" >> $name$ei.cstp
    done
    echo "\n$txh${txg}Created$txc${txg} test pack $txc$txh$name$ei.cstp$txc"
    exit 0
fi

# Make sure compile.sh exists
if [ ! -f "./compile.sh" ]
then
    echo "compile.sh does not exist"
    exit 1
fi

# Compile the program
./compile.sh >> /dev/null 2>&1
# Check if run.sh exited with a permission error
if [ $? -eq 126 ]; then
    echo "Permission denied on compile.sh. Fixing..."
    chmod +x ./compile.sh
    ./compile.sh
fi

# Make sure run.sh exists
if [ ! -f "./run.sh" ]
then
    echo "run.sh does not exist"
    exit 1
fi

# Run the tests
for i in "${matching_files[@]}"
do
    # Run the program
    ./run.sh ./in$i.txt > tmp.txt 2>err.txt

    # Check if run.sh exited with a permission error
    if [ $? -eq 126 ]; then
        echo "Permission denied on run.sh. Fixing..."
        chmod +x ./run.sh
        ./run.sh ./in$i.txt > tmp.txt 2>err.txt
    fi

    # Check if the output matches the expected output
    if diff './tmp.txt' "./out$i.txt" >/dev/null ; then
        echo "✅ Test $i passed!"
    else
        # If the output does not match, print the diff to make debugging easier, then exit
        if [ -s "err.txt" ]; then
            echo "💀💀💀 Test $i crashed! 💀💀💀"
            echo ===============================================
            cat tmp.txt
            cat err.txt
            echo ===============================================
        else
            echo "❌ Test $i failed!"
            if $diff; then
                echo "=======[ ${txr}Your Output${txc} | ${txg}Expected Output${txc} ]======="
                printf '%s\n' "$(git diff --no-index --color=always -U1000 ./tmp.txt ./out$i.txt | sed -r '1,5d')"
                echo ===============================================
            else
                echo \\nExpected output:\\n===============================================
                cat "./out$i.txt"
                echo ===============================================
                echo \\nActual output:\\n===============================================
                cat "./tmp.txt"
                echo ===============================================
            fi
        fi
        rm tmp.txt
        rm err.txt
        exit 1
    fi
done
if [ $# -eq 0 ]
then
    echo "✅ All tests passed!"
fi
rm tmp.txt
rm err.txt
