package frc.utils.buffers.RingBuffer;

import java.util.Iterator;
import java.util.NoSuchElementException;

public class RingBufferIterator<T> implements Iterator<T> {

	private final RingBuffer<T> ringBuffer;
	private final int endIndex;
	private int currentIndex;
	private boolean used;

	protected RingBufferIterator(RingBuffer<T> ringBuffer) {
		int insertions = ringBuffer.getInsertions();

		this.ringBuffer = ringBuffer;
		this.endIndex = ringBuffer.getCurrentIndex();
		this.currentIndex = insertions >= ringBuffer.size()
			? ringBuffer.getCurrentIndex()
			: ringBuffer.wrapIndex(ringBuffer.getCurrentIndex() - insertions);
		this.used = false;
	}

	@Override
	public boolean hasNext() {
		return ringBuffer.existsAtIndex(currentIndex) && !((this.used) && currentIndex == endIndex);
	}

	@Override
	public T next() throws NoSuchElementException {
		if (!hasNext()) {
			throw new NoSuchElementException("Ring buffer iterator exhausted");
		}
		T value = ringBuffer.get(currentIndex).get(); // the get operation is safe because we check if the value exists in hasNext()
		currentIndex = ringBuffer.wrapIndex(currentIndex + 1);
		this.used = true;
		return value;
	}

}
