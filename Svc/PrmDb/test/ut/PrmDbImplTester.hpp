/*
 * PrmDbImplTester.hpp
 *
 *  Created on: Mar 18, 2015
 *      Author: tcanham
 */

#ifndef PRMDB_TEST_UT_PRMDBIMPLTESTER_HPP_
#define PRMDB_TEST_UT_PRMDBIMPLTESTER_HPP_

#include <Svc/PrmDb/test/ut/PrmDbImplTesterCfg.hpp>
#include <Svc/PrmDb/PrmDbImpl.hpp>
#include <Os/File.hpp>

namespace Svc {

    class PrmDbImplTester {
        public:
            PrmDbImplTester(Svc::PrmDbImpl& inst);
            virtual ~PrmDbImplTester();

            void runNominalPopulate(void);
            void runNominalUpdate(void);
            void runNominalSerDes(void);
            void runNominalClear(void);
            void runNominalIterator(void);

            void runFullPopulate(void);
            void runInvalidSerialize(void);

        private:
            Svc::PrmDbImpl& m_impl;
    };

} /* namespace SvcTest */

#endif /* PRMDB_TEST_UT_PRMDBIMPLTESTER_HPP_ */
