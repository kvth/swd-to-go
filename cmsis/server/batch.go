package server

import "github.com/kvth/swd-to-go/cmsis/wire"

// Command batching: wire.CmdExecuteCommands packs several commands back to back
// into one request and expects their responses packed back to back into one
// response, and wire.CmdQueueCommands is the same with a promise that more is
// coming.

// ProcessCmd handles one request packet and fills in its response, returning how
// many request bytes it consumed and how many response bytes it produced.
func (s *Server) ProcessCmd(request []byte, response []byte) (consumed int, produced int) {
	if len(request) == 0 || len(response) == 0 {
		return 0, 0
	}

	// Nothing here defers work between packets, so queuing a batch is executing
	// it; the transport owes the client a response per packet either way.
	if request[0] == wire.CmdExecuteCommands || request[0] == wire.CmdQueueCommands {
		return s.executeCommands(request, response)
	}

	num := s.processCmd(request, response)
	return int((num >> 16) & 0xFFFF), int(num & 0xFFFF)
}

// executeCommands drains a batch. Each command in it sees only what the ones
// before it left of either packet, so a batch that runs out of room comes back
// short rather than coming back unsendable.
func (s *Server) executeCommands(request []byte, response []byte) (int, int) {
	if len(request) < 2 || len(response) < 2 {
		response[0] = wire.CmdInvalid
		return 1, 1
	}

	response[0] = request[0]
	count := int(request[1])
	response[1] = byte(count)

	reqPos, respPos := 2, 2
	for ; count > 0; count-- {
		if reqPos >= len(request) || respPos >= len(response) {
			break
		}

		num := s.processCmd(request[reqPos:], response[respPos:])
		taken := int((num >> 16) & 0xFFFF)
		used := int(num & 0xFFFF)
		if taken == 0 && used == 0 {
			break
		}
		// Clamping keeps a handler that miscounted from walking the batch past
		// the end of either packet.
		if taken > len(request)-reqPos {
			taken = len(request) - reqPos
		}
		if used > len(response)-respPos {
			used = len(response) - respPos
		}
		reqPos += taken
		respPos += used
	}

	return reqPos, respPos
}
