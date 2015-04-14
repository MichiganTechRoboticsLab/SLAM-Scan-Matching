#! /bin/bash
# 
#$ -cwd
#$ -j y
#$ -S /bin/bash
#$ -M dawonnac@mtu.edu
#$ -m abes
#$ -q long.q
#$ -t 1-14
#$ -hold_jid 1596
#$ -hard -l mem_free=1G
#$ -hard -l matlab_lic=1
#$ -notify
#

# Necessary variables
. /share/apps/bin/bashrc
. /share/apps/bin/an_functions.sh

# MATLAB R2014b
export MATLAB="/share/apps/matlab/R2014b"
export PATH="${PATH}:${MATLAB}/bin"

# Folder where the files are located
# Folder where the calculation will be done
export INIT_DIR="$PWD"

# Name of the MATLAB input file
export INPUT_FILE="mattest"
export ARRAY_JOB="_${SGE_TASK_ID}"

# Name of the MATLAB log file
export LOG_FILE="out"

# Calculation specific information
export LOCATION=`hostname | awk -F '.' '{print $1}'`
cat << EndOfFile > $INIT_DIR/job_info.$JOB_ID${ARRAY_JOB}

  Job ID               : $JOB_ID
  Username             : dawonnac
  Primary group        : thavens-users
  Login node           : $SGE_O_HOST
  Working directory    : $PWD
  Scratch directory    : $PWD
  Program              : MATLAB R2014b (serial)
  Input file           : mattest
  Queue                : long.q
  Array job            : Yes
  Task ID range        : ${SGE_TASK_ID} of 1-14
  Exclusive access     : No
  Dependent job ID     : 1596
  SMS notification     : No
  # of hosts           : 1
  # of processors      : 1
  Parent node          : $LOCATION
  Worker node          : $LOCATION
  Job submission time  : `sge_jst $JOB_ID `
  Job start time       : `date -R`
EndOfFile

# Start the timer
TIME_START=$(date +%s)

# Run MATLAB R2014b (serial)
# $MATLAB/bin/matlab -nodisplay -nosplash -singleCompThread -r "${INPUT_FILE}(${JOB_ID},${SGE_TASK_ID})" -logfile ../${JOB_ID}/${LOG_FILE}${ARRAY_JOB}.log
$MATLAB/bin/matlab -nodisplay -nosplash -singleCompThread -r "${INPUT_FILE}(${JOB_ID},${SGE_TASK_ID})" 

# End the timer
TIME_END=$(date +%s)

# Calculate time difference
TIME_TOTAL=`time2dhms $(( $TIME_END - $TIME_START ))`

cat << EndOfFile >> $INIT_DIR/job_info.$JOB_ID${ARRAY_JOB}
  Job end time         : `date -R`
  Total run time       : $TIME_TOTAL

EndOfFile
