using System;
using AWSIM_Script.Object;

namespace AWSIM.AWAnalysis.TraceExporter
{
	public class TraceCaptureConfig
	{
		public TraceCaptureConfig(CaptureStartingTime startingConfig, float savingTimeout)
		{
            TraceCaptureFrom = startingConfig;
            SavingTimeout = savingTimeout;
		}

		public CaptureStartingTime TraceCaptureFrom { get; set; }
		public float SavingTimeout { get; set; } = Simulation.DUMMY_SAVING_TIMEOUT;
	}

    public enum CaptureStartingTime
    {
	    AW_AUTO_MODE_READY, // when Autoware autonomous mode becomes ready, default
        AW_LOCALIZATION_INITIALIZED, // when Autoware localization is initialized
        AWSIM_STARTED // when AWSIM is started
    }
}

