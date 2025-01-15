package frc.robot.poseestimator.helpers;

import java.util.Iterator;

public class RingBufferIterator<T> implements Iterator<T> {

	private final RingBuffer<T> ringBuffer;
	private final int endIndex;
	private int currentIndex;
	private boolean used;

	RingBufferIterator(RingBuffer<T> ringBuffer) {
		this.ringBuffer = ringBuffer;
		int index = ringBuffer.getCurrentIndex();
		while(true) {
			if (--index < 0) {
				index = ringBuffer.size() - 1;
			}
			if (!ringBuffer.existsAtIndex(index)){
				index=ringBuffer.incWrapIndex(index);
				break;
			}
			if (index == ringBuffer.getCurrentIndex()){
				break;
			}
		}
		this.endIndex = ringBuffer.getCurrentIndex();
		this.currentIndex = index;
		this.used = false;
	}

	@Override
	public boolean hasNext() {
		return ringBuffer.existsAtIndex(currentIndex) && !((this.used) && currentIndex == endIndex);
	}

	@Override
	public T next() {
		if (!hasNext()) {
			return null;
		}
		T value = ringBuffer.getAtIndex(currentIndex).get(); // the get operation is safe because we check if the value exists in hasNext()
		currentIndex = ringBuffer.incWrapIndex(currentIndex);
		this.used = true;
		return value;
	}

}
