# To run dtron:
# * uppaal-tron-1.5-linux must be installed
# * Spread must be running (i.e., spread -n localhost)

# TRON_HOME environment variable pointing to un-zipped TRON distribution downloaded from: http://people.cs.aau.dk/~marius/tron
# 
export TRON_HOME=$(rospack find testit_dtron)/tron/uppaal-tron-1.5-linux

if [ ! -d "$TRON_HOME" ]; then
  echo "Unable to find TRON! Please point to un-zipped TRON distribution downloaded from: http://people.cs.aau.dk/~marius/tron"
  exit 127
fi

if [ "$#" -ne 2 ]; then
    echo "Usage: run_dtron_test.sh uppaal_model_xml_path time_unit_in_usec"
    exit
fi

java -jar $(rospack find testit_dtron)/dtron/dtron-4.14.jar -f $1 -u $2 -o 400000 -P eager
