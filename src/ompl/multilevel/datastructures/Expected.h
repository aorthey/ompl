#include <boost/outcome.hpp>

template <class T, class E>
using Expected = boost::outcome_v2::basic_result<T, E, boost::outcome_v2::policy::default_policy<T, E, void>>;
using boost::outcome_v2::failure;
using boost::outcome_v2::success;

