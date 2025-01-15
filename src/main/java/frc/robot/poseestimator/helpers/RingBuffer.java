package frc.robot.poseestimator.helpers;


import java.util.ArrayList;
import java.util.Iterator;
import java.util.Optional;

public class RingBuffer<T> implements Iterable<T> {

	private final ArrayList<Optional<T>> buffer;
	private int currentIndex;

	public RingBuffer(int bufferSize) {
		this.buffer = new ArrayList<>(bufferSize);
		for (int i = 0; i < bufferSize; i++) {
			buffer.add(Optional.empty());
		}
		this.currentIndex = 0;
	}

	public void insert(T data) {
		buffer.set(this.currentIndex++, Optional.of(data));
		if (currentIndex >= buffer.size()) {
			currentIndex = 0;
		}
	}

	public int size() {
		return buffer.size();
	}

	public int valuesAmount() {
		int filledSlots = 0;
		for (Optional<T> data : buffer) {
			if (data.isPresent()) {
				filledSlots++;
			}
		}
		return filledSlots;
	}

	public void clear() {
		for (int i = 0; i < size(); i++) {
			buffer.set(i, Optional.empty());
		}
	}

	int getCurrentIndex() {
		return currentIndex;
	}

	int incWrapIndex(int index) {
		if (++index >= size()) {
			index = 0;
		}
		return index;
	}

	boolean existsAtIndex(int index) {
		return buffer.get(index).isPresent();
	}

	Optional<T> getAtIndex(int index) {
		return buffer.get(index);
	}

	@Override
	public Iterator<T> iterator() {
		return new RingBufferIterator<>(this);
	}

}
