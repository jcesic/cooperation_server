#define PCL_ADD_POINT4D_MY \
		union { \
	float data[4]; \
	struct { \
			float x; \
			float y; \
			float z; \
	}; \
} EIGEN_ALIGN16; \
inline Eigen::Map<Eigen::Vector3f> getVector3fMap () { return (Eigen::Vector3f::Map (data)); } \
inline const Eigen::Map<const Eigen::Vector3f> getVector3fMap () const { return (Eigen::Vector3f::Map (data)); } \
inline Eigen::Map<Eigen::Vector4f, Eigen::Aligned> getVector4fMap () { return (Eigen::Vector4f::MapAligned (data)); } \
inline const Eigen::Map<const Eigen::Vector4f, Eigen::Aligned> getVector4fMap () const { return (Eigen::Vector4f::MapAligned (data)); } \
inline Eigen::Map<Eigen::Array3f> getArray3fMap () { return (Eigen::Array3f::Map (data)); } \
inline const Eigen::Map<const Eigen::Array3f> getArray3fMap () const { return (Eigen::Array3f::Map (data)); } \
inline Eigen::Map<Eigen::Array4f, Eigen::Aligned> getArray4fMap () { return (Eigen::Array4f::MapAligned (data)); } \
inline const Eigen::Map<const Eigen::Array4f, Eigen::Aligned> getArray4fMap () const { return (Eigen::Array4f::MapAligned (data)); }

struct _PointXYZ_MY{
	PCL_ADD_POINT4D_MY;  // This adds the members x,y,z which can also be accessed using the point (which is float[4])
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

struct PointXYZ_MY : public _PointXYZ_MY
{
	inline PointXYZ_MY()
	{
		x = y = z = 0.0f;
		data[3] = 1.0f;
	}
	inline PointXYZ_MY (float _x, float _y, float _z) { x = _x; y = _y; z = _z; data[3] = 1.0f; }
};

struct FieldMapping
{
	size_t serialized_offset;
	size_t struct_offset;
	size_t size;
};

typedef std::vector<FieldMapping> MsgFieldMapMY;

template <bool done = true> struct for_each_type_impl_moj
		{
	template<typename Iterator, typename LastIterator, typename F>
	static void execute (F) {}
		};

template <>
struct for_each_type_impl_moj<false>
{
	template<typename Iterator, typename LastIterator, typename F>
	static void execute (F f)
	{
		typedef typename boost::mpl::deref<Iterator>::type arg;

#if (defined _WIN32 && defined _MSC_VER)
		boost::mpl::aux::unwrap (f, 0).operator()<arg> ();
#else
		boost::mpl::aux::unwrap (f, 0).template operator()<arg> ();
#endif

		typedef typename boost::mpl::next<Iterator>::type iter;
		for_each_type_impl_moj<boost::is_same<iter, LastIterator>::value>
		::template execute<iter, LastIterator, F> (f);
	}
};

template<typename Sequence, typename F> inline void for_each_type_moj (F f){
	BOOST_MPL_ASSERT (( boost::mpl::is_sequence<Sequence> ));
	typedef typename boost::mpl::begin<Sequence>::type first;
	typedef typename boost::mpl::end<Sequence>::type last;
	for_each_type_impl_moj<boost::is_same<first, last>::value>::template execute<first, last, F> (f);
}

struct PCLPointFieldMY
{
  PCLPointFieldMY () : name (), offset (0), datatype (0), count (0)
  {}

  std::string name;
  uint32_t offset;
  uint8_t datatype;
  uint32_t count;

  enum PointFieldTypes {INT8 = 1, UINT8 = 2, INT16 = 3, UINT16 = 4, INT32 = 5, UINT32 = 6, FLOAT32 = 7, FLOAT64 = 8 };

public:
  typedef boost::shared_ptr<PCLPointFieldMY> Ptr;
  typedef boost::shared_ptr<PCLPointFieldMY const> ConstPtr;
}; // struct PCLPointField

template<typename PointT, typename Tag>
struct FieldMatchesMY
{
  bool operator() (const PCLPointFieldMY& field)
  {
    return (field.name == pcl::traits::name<PointT, Tag>::value &&
            field.datatype == pcl::traits::datatype<PointT, Tag>::value &&
            (field.count == pcl::traits::datatype<PointT, Tag>::size ||
             field.count == 0 && pcl::traits::datatype<PointT, Tag>::size == 1 /* see bug #821 */));
  }
};

inline bool
fieldOrderingMY (const FieldMapping& a, const FieldMapping& b)
{
  return (a.serialized_offset < b.serialized_offset);
}

template<typename PointT>
struct FieldMapperMY
{
	FieldMapperMY (const std::vector<PCLPointFieldMY>& fields,
			std::vector<FieldMapping>& map)
	: fields_ (fields), map_ (map)
	{
	}

	template<typename Tag> void
	operator () ()
	{
		int br = 0;
		BOOST_FOREACH (const PCLPointFieldMY& field, fields_)
    		{
			if (FieldMatchesMY<pcl::PointXYZ, Tag>()(field))
			{
				FieldMapping mapping;
				mapping.serialized_offset = field.offset;
				mapping.struct_offset = pcl::traits::offset<pcl::PointXYZ, Tag>::value;
				mapping.size = 4;
				map_.push_back (mapping);
				return;
			}
    		}
		// Disable thrown exception per #595: http://dev.pointclouds.org/issues/595
		//PCL_WARN ("Failed to find match for field '%s'.\n", traits::name<PointT, Tag>::value);*/
		//throw pcl::InvalidConversionException (ss.str ());
	}

	const std::vector<PCLPointFieldMY>& fields_;
	std::vector<FieldMapping>& map_;
};

