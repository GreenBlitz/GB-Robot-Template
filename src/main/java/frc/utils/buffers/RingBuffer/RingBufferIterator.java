package frc.utils.buffers.RingBuffer;

import java.util.Iterator;
import java.util.NoSuchElementException;

public class RingBufferIterator<T> implements Iterator<T> {

	private final BufferedQueue<T> bufferedQueue;
	private final int endIndex;
	private int currentIndex;
	private boolean used;

	protected RingBufferIterator(BufferedQueue<T> bufferedQueue) {
		int insertions = bufferedQueue.getInsertions();

		this.bufferedQueue = bufferedQueue;
		this.endIndex = bufferedQueue.getCurrentIndex();
		this.currentIndex = insertions >= bufferedQueue.size()
			? bufferedQueue.getCurrentIndex()
			: bufferedQueue.wrapIndex(bufferedQueue.getCurrentIndex() - insertions);
		this.used = false;
	}

	@Override
	public boolean hasNext() {
		return bufferedQueue.existsAtIndex(currentIndex) && !((this.used) && currentIndex == endIndex);
	}

	@Override
	public T next() throws NoSuchElementException {
		if (!hasNext()) {
			throw new NoSuchElementException("Ring buffer iterator exhausted");
		}
		T value = bufferedQueue.getAtIndex(currentIndex).get(); // the get operation is safe because we check if the value exists in hasNext()
		currentIndex = bufferedQueue.wrapIndex(currentIndex + 1);
		this.used = true;
		return value;
	}

}
