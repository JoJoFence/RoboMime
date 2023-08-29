using SSSoftworks.Data;

namespace AuthenticTeleoperation.Messages
{
    public sealed class MediaBufferPool : RecyclePool<MediaBuffer>
    {
        public MediaBufferPool( ObjectFactory factory ) : base( factory )
        {
        }
    }
}
