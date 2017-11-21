/**
 * \file   RosImgComponentImpl.hpp
 * \author Gene Merewether
 * \brief  Component that act as a wrapper for GNC NAV Image processing libraries
 *
 * \copyright
 * Copyright 2009-2016, by the California Institute of Technology.
 * ALL RIGHTS RESERVED.  United States Government Sponsorship
 * acknowledged. Any commercial use must be negotiated with the Office
 * of Technology Transfer at the California Institute of Technology.
 * <br /><br />
 * This software may be subject to U.S. export control laws and
 * regulations.  By accepting this document, the user agrees to comply
 * with all U.S. export laws and regulations.  User has the
 * responsibility to obtain export licenses, or other export authority
 * as may be required before exporting such information to foreign
 * countries or providing access to foreign persons.
 */


#ifndef ROSIMG_ROSIMGCOMPONENTIMPL_HPP_
#define ROSIMG_ROSIMGCOMPONENTIMPL_HPP_

#include <NAVOUTDOOR/RosImg/RosImgComponentAc.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>

namespace Navoutdoor {

    //! \class RosImgComponentImpl
    //! \brief Vision NAV component class
    //!
    //! This component acts as a wrapper for the GNC libraries by calling
    //! GNC library methods, and providing the interfaces to be integrated
    //! into the rest of the system.  In particular, it image data from the
    //! camera drivers, performs processing of that data, and sends image
    //! feature lists to the GncNav component.

    class RosImgComponentImpl : public RosImgComponentBase {

        public:

            //!  \brief Component constructor
            //!
            //!  The constructor initializes the state of the component.
            //!
            //!  Note: Making constructor explicit to prevent implicit
            //!  type conversion.
            //!
            //!  \param name the component instance name
#if FW_OBJECT_NAMES == 1
            explicit RosImgComponentImpl(const char* name);
#else
            RosImgComponentImpl();
#endif

            //!  \brief Component initialization routine
            //!
            //!  The initialization function calls the initialization
            //!  routine for the base class.
            //!
            //!  \param queueDepth the depth of the message queue for the component
            void init(NATIVE_INT_TYPE queueDepth, NATIVE_INT_TYPE instance);

            //!  \brief Component destructor
            //!
            ~RosImgComponentImpl();

        PROTECTED:

        PRIVATE:

            // ----------------------------------------------------------------------
            // Prohibit Copying
            // ----------------------------------------------------------------------

            /*! \brief Copy constructor
             *
             */
            RosImgComponentImpl(const RosImgComponentImpl&);

            /*! \brief Copy assignment operator
             *
             */
            RosImgComponentImpl& operator=(const RosImgComponentImpl&);

            // ----------------------------------------------------------------------
            // Constants/Types
            // ----------------------------------------------------------------------

            // ----------------------------------------------------------------------
            // Member Functions
            // ----------------------------------------------------------------------

            // ----------------------------------------------------------------------
            // Handlers to implement for typed input ports
            // ----------------------------------------------------------------------

            //! Handler for input port CameraData
            //
            virtual void ImageRecv_handler(
                NATIVE_INT_TYPE portNum, /*!< The port number*/
                Fw::Buffer &fwBuffer
            );

            // ----------------------------------------------------------------------
            // Helper Methods
            // ----------------------------------------------------------------------

            // ----------------------------------------------------------------------
            // Member Variables
            // ----------------------------------------------------------------------

            ros::NodeHandle* m_nodeHandlePtr;

            image_transport::ImageTransport* m_imageTransportPtr;

            image_transport::Publisher m_pubImage;

    }; // end class definition

} // end namespace

#endif /* ROSIMG_ROSIMGCOMPONENTIMPL_HPP_ */
