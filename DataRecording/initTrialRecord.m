function trialData = initTrialRecord(trialDataFile)

trialData = matfile(trialDataFile, 'Writable', true);
trialData.dt = 0;
trialData.dist = 0;
trialData.energy = 0;
trialData.voltage = 0;
trialData.current = 0;
trialData.totalEnergy = 0;
trialData.pdt = 0;