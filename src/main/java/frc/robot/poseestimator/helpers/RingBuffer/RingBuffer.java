package frc.robot.poseestimator.helpers.RingBuffer;


import java.util.ArrayList;
import java.util.Iterator;
import java.util.Optional;

public class RingBuffer<T> implements Iterable<T> {

	private final ArrayList<Optional<T>> buffer;
	private int currentIndex;
	private int insertions;

	public RingBuffer(int bufferSize) {
		this.buffer = new ArrayList<>(bufferSize);
		for (int i = 0; i < bufferSize; i++) {
			buffer.add(Optional.empty());
		}
		this.currentIndex = 0;
	}

	public void insert(T data) {
		insertions++;
		buffer.set(this.currentIndex++, Optional.of(data));
		if (currentIndex >= buffer.size()) {
			currentIndex = 0;
		}
	}

	public int size() {
		return buffer.size();
	}

	public int filledSlots() {
		return insertions >= size() ? size() - 1 : insertions;
	}

	public void clear() {
		insertions = 0;
		currentIndex = 0;
		for (int i = 0; i < size(); i++) {
			buffer.set(i, Optional.empty());
		}
	}

	protected int getCurrentIndex() {
		return currentIndex;
	}

	protected int wrapIndex(int index) {
		index = index % size();
		if (index < 0) {
			index += size();
		}
		return index;
	}

	protected int getInsertions() {
		return insertions;
	}

	protected boolean existsAtIndex(int index) {
		return buffer.get(index).isPresent();
	}

	protected Optional<T> getAtIndex(int index) {
		return buffer.get(index);
	}

	@Override
	public Iterator<T> iterator() {
		return new RingBufferIterator<>(this);
	}

}
