function launch_fl()
{
OUTPUT_FOLDER=$1
SCRIPT=launcher_imola_race_optimization
REFERENCE_LAP_SCRIPT=imola_race_lap

if [[ -d "$OUTPUT_FOLDER" ]]
then
    echo "[ABORTED] $OUTPUT_FOLDER exists on your filesystem."
    return 99
fi

mkdir -p $OUTPUT_FOLDER
mkdir -p $OUTPUT_FOLDER/input
mkdir -p $OUTPUT_FOLDER/output
mkdir -p $OUTPUT_FOLDER/data

# Copy the input script into the output folder
cp $REFERENCE_LAP_SCRIPT.m $OUTPUT_FOLDER/input
cp $SCRIPT.m $OUTPUT_FOLDER/input

sbatch --job-name=$OUTPUT_FOLDER --output="$OUTPUT_FOLDER/output/output.log" --error="$OUTPUT_FOLDER/output/output.log" << EOF
#!/bin/bash
#SBATCH --partition=standard
#SBATCH --ntasks=1 --cpus-per-task=1
#SBATCH --mem-per-cpu=2G
#SBATCH --time=24:00:00

. load_run_env.sh
srun matlab -r "output_folder='$OUTPUT_FOLDER'; $REFERENCE_LAP_SCRIPT; $SCRIPT; exit"
EOF
}
