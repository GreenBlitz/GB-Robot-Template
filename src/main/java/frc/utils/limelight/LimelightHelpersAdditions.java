package frc.utils.limelight;

public class LimelightHelpersAdditions {

	public record RawTarget(double txnc, double tync, double ta) {

		public RawTarget() {
			this(0, 0, 0);
		}

		@Override
		public boolean equals(Object obj) {
			if (this == obj)
				return true;
			if (obj == null || getClass() != obj.getClass())
				return false;
			RawTarget other = (RawTarget) obj;
			return Double.compare(txnc, other.txnc) == 0 && Double.compare(tync, other.tync) == 0 && Double.compare(ta, other.ta) == 0;
		}

	}

	public static RawTarget[] getRawTargets(String limelightName) {
		var entry = LimelightHelpers.getLimelightNTTableEntry(limelightName, "rawtargets");
		var rawTargetArray = entry.getDoubleArray(new double[0]);
		int valsPerEntry = 3;
		if (rawTargetArray.length % valsPerEntry != 0) {
			return new RawTarget[0];
		}

		int numTargets = rawTargetArray.length / valsPerEntry;
		RawTarget[] rawTargets = new RawTarget[numTargets];

		for (int i = 0; i < numTargets; i++) {
			int baseIndex = i * valsPerEntry; // Starting index for this target's data
			double txnc = extractArrayEntry(rawTargetArray, baseIndex);
			double tync = extractArrayEntry(rawTargetArray, baseIndex + 1);
			double ta = extractArrayEntry(rawTargetArray, baseIndex + 2);

			rawTargets[i] = new RawTarget(txnc, tync, ta);
		}

		return rawTargets;
	}

	public static boolean getIsConnected(String limelightName) {
		return LimelightHelpers.getLimelightNTTable(limelightName).containsKey("getpipe");
	}

	private static double extractArrayEntry(double[] inData, int position) {
		if (inData.length < position + 1) {
			return 0;
		}
		return inData[position];
	}

}
