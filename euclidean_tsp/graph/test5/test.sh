# Get the name of the current file
filename=$(basename "$0")

echo "This version is deprecated. Updating to latest version..."

# check for updated version on server
wget -q https://gist.github.com/ajsacco/dce5fb77d045a051fd2582e3592ad315/raw -O $filename.new
if [ $? -eq 0 ]; then
    if ! diff -q $filename $filename.new >/dev/null ; then
        rm $filename
        mv $filename.new $filename
        chmod +x $filename
        echo "Updated successfully."
        exit 0
    fi
    rm $filename.new
    echo "New file is identical to this file. This should be impossible. Please tell me (Aidan) about this"
    exit 1
fi
rm $filename.new
echo "Error downloading update. Check your network connection."
exit 1