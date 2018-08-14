# To run dtron:
# * uppaal-tron-1.5-linux must be installed
# * Spread must be running (i.e., spread -n localhost)

# TRON_HOME environment variable pointing to un-zipped TRON distribution downloaded from: http://people.cs.aau.dk/~marius/tron
# 
export TRON_HOME=../tron/uppaal-tron-1.5-linux

if [ "$#" -ne 1 ]; then
    echo "Usage: run_dtron_test.sh uppaal_model_xml_path"
    exit
fi

java -jar dtron-4.14.jar -f $1 -u 100000 -o 400000 -P eager 
