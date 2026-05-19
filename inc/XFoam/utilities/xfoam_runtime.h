#ifndef XFoam_Utilities_Runtime_H_
#define XFoam_Utilities_Runtime_H_

// 对标 OpenFOAM db/runTimeSelection/construction/runTimeSelectionTables.H
// 命名见 doc/foam_code.md：函数式宏为 XFoam_ + lowerCamel（如 XFoam_declareRunTimeSelectionTable）。
// 首参 SmartPtr 在 XFoam 中通常填 XFoam_AutoPtr。

#include "XFoam/utilities/xfoam_types.h"
#include "XFoam/utilities/xfoam_autoptr.h"
#include "XFoam/utilities/xfoam_hash.h"

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

//- 声明 run-time selection 表（基类侧：函数指针表存构造函数）。对标 OpenFOAM declareRunTimeSelectionTable。
#define XFoam_declareRunTimeSelectionTable(SmartPtr, baseType, argNames, argList, parList) \
	\
	typedef SmartPtr<baseType> (*argNames##ConstructorPtr)argList; \
	typedef XFoam_HashTable<argNames##ConstructorPtr, XFoam_Word> argNames##ConstructorTable; \
	static argNames##ConstructorTable* argNames##ConstructorTablePtr_; \
	static void construct##argNames##ConstructorTables(); \
	static void destroy##argNames##ConstructorTables(); \
	template<class baseType##Type> \
	class add##argNames##ConstructorToTable \
	{ \
	public: \
		static SmartPtr<baseType> New argList \
		{ \
			return SmartPtr<baseType>(new baseType##Type parList); \
		} \
		add##argNames##ConstructorToTable(const XFoam_Word& lookup = baseType##Type::typeName) \
		{ \
			construct##argNames##ConstructorTables(); \
			if (!argNames##ConstructorTablePtr_->insert(lookup, New)) \
			{ \
				std::cerr << "Duplicate entry " << static_cast<const XFoam_String&>(lookup) \
						  << " in runtime selection table " << #baseType << std::endl; \
			} \
		} \
		~add##argNames##ConstructorToTable() { destroy##argNames##ConstructorTables(); } \
	}; \
	template<class baseType##Type> \
	class addRemovable##argNames##ConstructorToTable \
	{ \
		const XFoam_Word& lookup_; \
	public: \
		static SmartPtr<baseType> New argList \
		{ \
			return SmartPtr<baseType>(new baseType##Type parList); \
		} \
		addRemovable##argNames##ConstructorToTable(const XFoam_Word& lookup = baseType##Type::typeName) \
			: lookup_(lookup) \
		{ \
			construct##argNames##ConstructorTables(); \
			(void)argNames##ConstructorTablePtr_->set(lookup, New); \
		} \
		~addRemovable##argNames##ConstructorToTable() \
		{ \
			if (argNames##ConstructorTablePtr_) \
			{ \
				(void)argNames##ConstructorTablePtr_->erase(lookup_); \
			} \
		} \
	};

//- 声明 run-time selection（派生类侧：表项为 “New” 工厂指针）。对标 OpenFOAM declareRunTimeNewSelectionTable。
#define XFoam_declareRunTimeNewSelectionTable(SmartPtr, baseType, argNames, argList, parList) \
	\
	typedef SmartPtr<baseType> (*argNames##ConstructorPtr)argList; \
	typedef XFoam_HashTable<argNames##ConstructorPtr, XFoam_Word> argNames##ConstructorTable; \
	static argNames##ConstructorTable* argNames##ConstructorTablePtr_; \
	static void construct##argNames##ConstructorTables(); \
	static void destroy##argNames##ConstructorTables(); \
	template<class baseType##Type> \
	class add##argNames##ConstructorToTable \
	{ \
	public: \
		static SmartPtr<baseType> New##baseType argList \
		{ \
			return SmartPtr<baseType>(baseType##Type::New parList.ptr()); \
		} \
		add##argNames##ConstructorToTable(const XFoam_Word& lookup = baseType##Type::typeName) \
		{ \
			construct##argNames##ConstructorTables(); \
			if (!argNames##ConstructorTablePtr_->insert(lookup, New##baseType)) \
			{ \
				std::cerr << "Duplicate entry " << static_cast<const XFoam_String&>(lookup) \
						  << " in runtime selection table " << #baseType << std::endl; \
			} \
		} \
		~add##argNames##ConstructorToTable() { destroy##argNames##ConstructorTables(); } \
	}; \
	template<class baseType##Type> \
	class addRemovable##argNames##ConstructorToTable \
	{ \
		const XFoam_Word& lookup_; \
	public: \
		static SmartPtr<baseType> New##baseType argList \
		{ \
			return SmartPtr<baseType>(baseType##Type::New parList.ptr()); \
		} \
		addRemovable##argNames##ConstructorToTable(const XFoam_Word& lookup = baseType##Type::typeName) \
			: lookup_(lookup) \
		{ \
			construct##argNames##ConstructorTables(); \
			(void)argNames##ConstructorTablePtr_->set(lookup, New##baseType); \
		} \
		~addRemovable##argNames##ConstructorToTable() \
		{ \
			if (argNames##ConstructorTablePtr_) \
			{ \
				(void)argNames##ConstructorTablePtr_->erase(lookup_); \
			} \
		} \
	};

#define XFoam_defineRunTimeSelectionTableConstructor(baseType, argNames) \
	void baseType::construct##argNames##ConstructorTables() \
	{ \
		static bool constructed = false; \
		if (!constructed) \
		{ \
			constructed = true; \
			baseType::argNames##ConstructorTablePtr_ = new baseType::argNames##ConstructorTable; \
		} \
	}

#define XFoam_defineRunTimeSelectionTableDestructor(baseType, argNames) \
	void baseType::destroy##argNames##ConstructorTables() \
	{ \
		if (baseType::argNames##ConstructorTablePtr_) \
		{ \
			delete baseType::argNames##ConstructorTablePtr_; \
			baseType::argNames##ConstructorTablePtr_ = nullptr; \
		} \
	}

#define XFoam_defineRunTimeSelectionTablePtr(baseType, argNames) \
	baseType::argNames##ConstructorTable* baseType::argNames##ConstructorTablePtr_ = nullptr

#define XFoam_defineRunTimeSelectionTable(baseType, argNames) \
	XFoam_defineRunTimeSelectionTablePtr(baseType, argNames); \
	XFoam_defineRunTimeSelectionTableConstructor(baseType, argNames); \
	XFoam_defineRunTimeSelectionTableDestructor(baseType, argNames)

#define XFoam_defineTemplateRunTimeSelectionTable(baseType, argNames) \
	template<> \
	XFoam_defineRunTimeSelectionTablePtr(baseType, argNames); \
	template<> \
	XFoam_defineRunTimeSelectionTableConstructor(baseType, argNames); \
	template<> \
	XFoam_defineRunTimeSelectionTableDestructor(baseType, argNames)

#define XFoam_defineTemplatedRunTimeSelectionTableConstructor(baseType, argNames, Targ) \
	void baseType<Targ>::construct##argNames##ConstructorTables() \
	{ \
		static bool constructed = false; \
		if (!constructed) \
		{ \
			constructed = true; \
			baseType<Targ>::argNames##ConstructorTablePtr_ = new baseType<Targ>::argNames##ConstructorTable; \
		} \
	}

#define XFoam_defineTemplatedRunTimeSelectionTableDestructor(baseType, argNames, Targ) \
	void baseType<Targ>::destroy##argNames##ConstructorTables() \
	{ \
		if (baseType<Targ>::argNames##ConstructorTablePtr_) \
		{ \
			delete baseType<Targ>::argNames##ConstructorTablePtr_; \
			baseType<Targ>::argNames##ConstructorTablePtr_ = nullptr; \
		} \
	}

#define XFoam_defineTemplatedRunTimeSelectionTablePtr(baseType, argNames, Targ) \
	baseType<Targ>::argNames##ConstructorTable* baseType<Targ>::argNames##ConstructorTablePtr_ = nullptr

#define XFoam_defineTemplatedRunTimeSelectionTable(baseType, argNames, Targ) \
	template<> \
	XFoam_defineTemplatedRunTimeSelectionTablePtr(baseType, argNames, Targ); \
	template<> \
	XFoam_defineTemplatedRunTimeSelectionTableConstructor(baseType, argNames, Targ); \
	template<> \
	XFoam_defineTemplatedRunTimeSelectionTableDestructor(baseType, argNames, Targ)

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

#endif
