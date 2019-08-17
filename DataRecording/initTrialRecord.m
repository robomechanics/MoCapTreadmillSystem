function trialData = initTrialRecord(trialDataFile)

trialData = matfile(trialDataFile, 'Writable', true);
trialData.dt = 0;
trialData.dist = 0;
trialData.totalEnergy = 0;
