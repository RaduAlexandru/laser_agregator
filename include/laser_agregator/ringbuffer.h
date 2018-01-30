// ringbuffer.h
// Author: Markus Redeker

#pragma warning(disable:4786)

#ifndef __RINGBUFFER_H__
#define __RINGBUFFER_H__

// Use:
// ringbuffer r;
// r.push(); r.back() = new_element;
// oldest_element = r.front(); r.pop();

template<typename ET, int S, typename ST=int>
class ringbuffer
{
public:
	typedef ET value_type;
    typedef ST size_type;

	ringbuffer()
	{
		clear();
	}

	~ringbuffer() {}

	size_type size()     const { return m_size; }
	size_type max_size() const { return S; }

	bool empty() const	{ return m_size == 0; }
	bool full()  const	{ return m_size == S; }

	value_type& front() { return m_buffer[m_front]; }
	value_type& back() { return m_buffer[m_back]; }

	void clear() {
		m_size = 0;
		m_front = 0;
		m_back  = S - 1;
	}

	void push()	{
		m_back = (m_back + 1) % S;
		if( size() == S )
			m_front = (m_front + 1) % S;
		else
			m_size++;
	}

	void push(const value_type& x) {
		push();
		back() = x;

		//update the cumulative average and the exponential average. (If it's the first value we push into the ring, then just set it to the value)
		if(std::is_arithmetic<value_type >::value){
			if(first_value){
				m_avg= x;
				m_exp_avg= x;
 			}else{
				m_avg=	m_avg + 1.0/(nr_inserted_values+1)*(x-m_avg);
				m_exp_avg = m_exp_avg + alpha*(x-m_exp_avg);
			}
		}
		first_value=false;
		nr_inserted_values++;

	}

	void pop() {
		if( m_size > 0  ) {
			m_size--;
			m_front = (m_front + 1) % S;
		}
	}

	void back_erase(const size_type n) {
		if( n >= m_size )
			clear();
		else {
			m_size -= n;
			m_back = (m_front + m_size - 1) % S;
		}
	}

	void front_erase(const size_type n) {
		if( n >= m_size )
			clear();
		else {
			m_size -= n;
			m_front = (m_front + n) % S;
		}
	}

	value_type* data(){
		return m_buffer;
	}

	size_type get_front_idx(){
		return m_front;
	}

	size_type get_back_idx(){
		return m_back;
	}

	double exp_avg(){
		return m_exp_avg;
	}

	double avg(){
		return m_avg;
	}

protected:

	value_type m_buffer[S];

	size_type m_size;

	size_type m_front;
	size_type m_back;

	int nr_inserted_values=0;
	double alpha=0.07;
	double m_avg=0;
	double m_exp_avg=0;

	bool first_value=true;
};


#endif // __RINGBUFFER_H__
