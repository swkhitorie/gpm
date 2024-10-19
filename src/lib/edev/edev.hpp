
#ifndef __EDEV_H_
#define __EDEV_H_

namespace ESAF
{

/*!
 * @brief Fundamental base class for all physical drivers
 * This class provides the basic driver template for devices
 */
class EDev
{
  public:
    // no copy, assignment, move, move assignment
    EDev(const EDev &) = delete;
    EDev &operator=(const EDev &) = delete;
    EDev(EDev &&)                 = delete;
    EDev &operator=(EDev &&) = delete;

    virtual ~EDev() = default;

    enum ResType
    {
        ENONE,
        EOK
    };

    enum RWStream
    {
        STREAMIN,
        STREAMOUT
    };

    enum RWWay
    {
        BLOCK,
        NONBLOCK,
    };

    /**
     * @brief Initialise the driver and make it ready for use.
     * @return EOK if the driver initialized OK
     */
    virtual int init() { return ENONE; }

    /**
     * @brief update device state and tx/rx data
     * @return EOK if the driver update OK
     */
    virtual int update() { return ENONE; }

    /**
     * @brief Read directly from the device buffer.
     * @param pdata The buffer into which the read values should be placed.
     * @param count The number of items to read.
     * @return  The number of items read on success
     */
    virtual int read(void *pdata, unsigned int count) { return ENONE; }

    /**
     * @brief Read directly from the peripheral.
     * @param pdata The buffer into which the read values should be placed.
     * @param count The number of items to read.
     * @param rwway See RWWay
     * @return  The number of items read on success
     */
    virtual int read_through(void *pdata, unsigned int count, int rwway) { return ENONE; }

    /**
     * @brief Write directly to the device buffer.
     * @param pdata The buffer from which values should be read.
     * @param count The number of items to write.
     * @return  The number of items written on success
     */
    virtual int write(const void *pdata, unsigned int count) { return ENONE; }

    /**
     * @brief Write directly to the peripheral.
     * @param pdata The buffer from which values should be read.
     * @param count The number of items to write.
     * @param rwway See RWWay
     * @return  The number of items written on success
     */
    virtual int write_through(const void *pdata, unsigned int count, int rwway) { return ENONE; }

    /**
     * @brief query in device rx buffer.
     * @param offset The device rx buffer offset at which to start query.
     * @param pdata The buffer into which the query values should be placed.
     * @param count The number of items to read.
     * @return  The number of items query on success
     */
    virtual int query(unsigned int offset, void *pdata, unsigned int count) { return ENONE; }

    /**
     * @brief update device buffer
     * @param rwstream see RWStream,
     * 			STREAMIN - update peripheral data into device rx buffer
     *  		STREAMOUT - update device tx buffer into peripheral
     */
    virtual void flush(int rwstream) {}

    /**
     * @brief clear device buffer
     * @param rwstream see RWStream,
     * 			STREAMIN - clear device rx buffer
     *  		STREAMOUT - clear device tx buffer
     */
    virtual void eclear(int rwstream) {}

    /**
     * @brief get device buffer size
     * @param rwstream see RWStream,
     * 			STREAMIN - get device rx buffer data size
     *  		STREAMOUT - get device tx buffer data size
     * @return size
     */
    virtual unsigned int esize(int rwstream) { return 0; }

    /**
     * @brief Perform a device-specific operation.
     * @param operation	The operation to perform.
     * @param arg		An argument to the operation.
     * @return positive value on success, negative value on error,
     */
    virtual int ioctl(unsigned int operation, unsigned int &arg) { return ENONE; }

  protected:
    bool _init{false};

    EDev() = default;
};

}  // namespace ESAF

#endif  // edev_H
