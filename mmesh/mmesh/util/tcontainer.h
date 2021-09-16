#ifndef MMESH_TCONTAINER_1620036442293_H
#define MMESH_TCONTAINER_1620036442293_H
#include <list>
#include <algorithm>

namespace mmesh
{
	template<class T>
	class TContainer
	{
	public:
		typedef typename std::list<T*>::iterator TIterator;
	public:
		TContainer() {}
		~TContainer() {}

		void addElement(T* element)
		{
			if (!element)
				return;

			TIterator it = std::find(m_elements.begin(), m_elements.end(), element);
			if (it != m_elements.end())
				return;

			m_elements.push_back(element);
		}

		void removeElement(T* element)
		{
			if (!element)
				return;

			TIterator it = std::find(m_elements.begin(), m_elements.end(), element);
			if (it == m_elements.end())
				return;

			m_elements.erase(it);
		}

		template<class Callback>
		void notify(Callback c)
		{
			for (TIterator it = m_elements.begin(); it != m_elements.end(); ++it)
				c(*it);
		}

	protected:
		std::list<T*> m_elements;
	};
}

#endif // MMESH_TCONTAINER_1620036442293_H