diff=true
txtgrn=$(tput setaf 2)
txtred=$(tput setaf 1)
txtrst=$(tput sgr0)


# check for updated version on server
wget -q https://files.asacco.dev/scripts/test.sh -O test.sh.new
if [ $? -eq 0 ]; then
    if ! diff -q test.sh test.sh.new >/dev/null ; then
        echo "New version of test.sh available. Updating..."
        rm test.sh
        mv test.sh.new test.sh
        chmod +x test.sh
        echo "Updated successfully. Please run ./test.sh again."
        exit 0
    fi
    rm test.sh.new
fi


# Make sure run.sh exists
if [ ! -f "./run.sh" ]
then
    echo "run.sh does not exist"
    exit 1
fi

matching_files=()

if [ $# -eq 0 ]
# If no arguments are given, run all tests
then
    for file in in*.txt
    do
        # Extract the number i from the file name
        i=$(echo "$file" | sed -n -E 's/^in([0-9]+)\.txt$/\1/p')
        out_file="out$i.txt"

        # Check if there exists a corresponding out_file
        if [ -f "./$out_file" ]
        then
            matching_files+=("$i")
        else
            echo "No matching output file for $file"
            exit 1
        fi
    done
# Otherwise, run the tests specified by the arguments
else
    matching_files=("$@")

    # Verify that the files exist
    for i in "${matching_files[@]}"
    do
        if [ ! -f "./in$i.txt" ]
        then
            echo "File in$i.txt does not exist"
            exit 1
        fi
        
        if [ ! -f "./out$i.txt" ]
        then
            echo "File out$i.txt does not exist"
            exit 1
        fi
    done
fi

for i in "${matching_files[@]}"
do
    # Add newline to end of file if it doesn't exist
    if [ "$(tail -c 1 out$i.txt)" != "" ]; then
        echo >> out$i.txt
    fi
done

# Run the tests
for i in "${matching_files[@]}"
do
    # Run the program
    ./run.sh ./in$i.txt > tmp.txt 2>err.txt

    # Check if run.sh exited with a permission error
    if [ $? -eq 126 ]; then
        echo "Permission denied on run.sh. Please run chmod +x run.sh"
        exit 1
    fi

    # Check if the output matches the expected output
    if diff './tmp.txt' "./out$i.txt" >/dev/null ; then
        echo "✅ Test $i passed!"
    else
        # If the output does not match, print the diff to make debugging easier, then exit
        if [ -s "err.txt" ]; then
            echo "💀💀💀 Test $i crashed! 💀💀💀"
            echo ===============================================
            cat err.txt
            echo ===============================================
        else
            echo "❌ Test $i failed!"
            if $diff; then
                echo "=======[ ${txtred}Your Output${txtrst} | ${txtgrn}Expected Output${txtrst} ]======="
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
