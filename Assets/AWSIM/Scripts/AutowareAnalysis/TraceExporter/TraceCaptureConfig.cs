using System;
namespace AWSIM.AWAnalysis.TraceExporter
{
	public class TraceCaptureConfig
	{
		public TraceCaptureConfig(CaptureStartingTime startingConfig)
		{
            this.TraceCaptureFrom = startingConfig;
		}

		public CaptureStartingTime TraceCaptureFrom { get; set; }

        // how many frame per second
    }

    public enum CaptureStartingTime
    {
        AW_LOCALIZATION_INITIALIZED, // when Autoware localization is initialized, default
        AWSIM_STARTED // when AWSIM is started
    }
}

